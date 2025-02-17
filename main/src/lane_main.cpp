
#include <cstddef>
#include <cstdint>
#include <etl/map.h>
#include <etl/random.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <NimBLEDevice.h>
#include <mutex>
#include <memory.h>
#include <RadioLib.h>
#include <sdkconfig.h>
#include "EspHal.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal.h"
#include "utils.h"
#include "Lane.h"
#include "whitelist.h"
#include "common.h"
#include "ScanCallback.h"
#include "hr_lora.h"
#include "ble_hr_data.h"

//#define DEBUG_SPEED
void *rf_receive_data = nullptr;
struct rf_receive_data_t {
  EventGroupHandle_t evt_grp = nullptr;
};

constexpr auto RecvEvt = BIT0;

constexpr auto send_lk_timeout_tick = 100;
constexpr auto MAX_DEVICE_COUNT     = 16;
using repeater_t                    = HrLoRa::repeater_status::t;
using device_name_map_t             = etl::flat_map<int, repeater_t, MAX_DEVICE_COUNT>;

struct handle_message_callbacks_t {
  std::function<std::optional<HrLoRa::hr_device::t>(int)> get_device_by_key;
  /// return true if the device is updated successfully, otherwise a key change is requested
  std::function<bool(repeater_t)> update_device;
  std::function<void(std::string name, int hr)> on_hr_data;
  std::function<void(uint8_t *data, size_t size)> rf_send;
};

void handle_message(uint8_t *pdata, size_t size, const handle_message_callbacks_t &callbacks) {
  static constexpr auto TAG = "handle_message";
  const bool callback_ok          = callbacks.get_device_by_key != nullptr &&
                     callbacks.update_device != nullptr &&
                     callbacks.rf_send != nullptr &&
                     callbacks.on_hr_data != nullptr;
  if (!callback_ok) {
    ESP_LOGE(TAG, "bad callback");
    return;
  }
  static auto rng = etl::random_xorshift(esp_random());
  const auto magic      = pdata[0];
  switch (magic) {
    case HrLoRa::hr_data::magic: {
      if (const auto hr_data_ = HrLoRa::hr_data::unmarshal(pdata, size)) {
        auto p_dev = callbacks.get_device_by_key(hr_data_->key);
        if (!p_dev) {
          ESP_LOGW(TAG, "no name for key %d", hr_data_->key);
          return;
        }
        callbacks.on_hr_data(p_dev->name, hr_data_->hr);
      }
      break;
    }
    case HrLoRa::named_hr_data::magic: {
      if (auto hr_data_ = HrLoRa::named_hr_data::unmarshal(pdata, size)) {
        auto dev_ = callbacks.get_device_by_key(hr_data_->key);
        if (!dev_) {
          ESP_LOGW(TAG, "no addr for key %d", hr_data_->key);
          return;
        }
        if (!std::equal(dev_->addr.begin(), dev_->addr.end(), hr_data_->addr.begin())) {
          ESP_LOGW(TAG, "addr mismatch %s and %s",
                   utils::toHex(dev_->addr.data(), dev_->addr.size()).c_str(),
                   utils::toHex(hr_data_->addr.data(), hr_data_->addr.size()).c_str());
          return;
        }
        if (const auto name = dev_->name; name.empty()) {
          auto addr_str = utils::toHex(hr_data_->addr.data(), hr_data_->addr.size());
          callbacks.on_hr_data(addr_str, hr_data_->hr);
        } else {
          callbacks.on_hr_data(name, hr_data_->hr);
        }
      }
      break;
    }
    case HrLoRa::repeater_status::magic: {
      if (const auto response_ = HrLoRa::repeater_status::unmarshal(pdata, size)) {
        if (const bool ok = callbacks.update_device(*response_); !ok) {
          // request a key change
          auto new_key = static_cast<uint8_t>(rng.range(0, 255));
          // TODO: Handle the case where all keys are in use and a new one cannot be generated
          constexpr auto limit = MAX_DEVICE_COUNT;
          auto counter         = 0;
          while (callbacks.get_device_by_key(new_key).has_value()) {
            new_key = rng.range(0, 255);
            if (++counter > limit) {
              ESP_LOGE(TAG, "failed to generate a unique key");
              return;
            }
          }
          const auto req = HrLoRa::set_name_map_key::t{
              .addr = response_->repeater_addr,
              .key  = response_->key,
          };
          uint8_t buf[16] = {0};
          auto sz         = HrLoRa::set_name_map_key::marshal(req, buf, sizeof(buf));
          if (sz == 0) {
            ESP_LOGE(TAG, "failed to marshal");
            return;
          }
          callbacks.rf_send(buf, sz);
        }
      } else {
        ESP_LOGE(TAG, "failed to unmarshal repeater_status");
      }
      break;
    }
    case HrLoRa::query_device_by_mac::magic:
    case HrLoRa::set_name_map_key::magic: {
      // leave out intentionally
      break;
    }
    default: {
      ESP_LOGW(TAG, "unknown magic: %d", magic);
    }
  }
}

/**
 * @brief try to transmit the data
 * @note would block until the transmission is done and will start receiving after that
 */
void try_transmit(uint8_t *data, size_t size,
                  SemaphoreHandle_t lk, TickType_t timeout_tick,
                  LLCC68 &rf) {
  const auto TAG = "try_transmit";
  if (xSemaphoreTake(lk, timeout_tick) != pdTRUE) {
    ESP_LOGE(TAG, "failed to take rf_lock; no transmission happens;");
    return;
  }
  if (const auto err = rf.transmit(data, size); err == RADIOLIB_ERR_NONE) {
    // ok
  } else if (err == RADIOLIB_ERR_TX_TIMEOUT) {
    ESP_LOGW(TAG, "tx timeout; please check the busy pin;");
  } else {
    ESP_LOGE(TAG, "failed to transmit, code %d", err);
  }
  rf.standby();
  rf.startReceive();
  xSemaphoreGive(lk);
}

size_t try_receive(uint8_t *buf, size_t max_size,
                   SemaphoreHandle_t lk, TickType_t timeout_tick,
                   LLCC68 &rf) {
  const auto TAG = "try_receive";
  // https://www.freertos.org/a00122.html
  if (xSemaphoreTake(lk, timeout_tick) != pdTRUE) {
    ESP_LOGE(TAG, "failed to take rf_lock");
    return 0;
  }
  auto length = rf.getPacketLength(true);
  if (length > max_size) {
    ESP_LOGE(TAG, "packet length %d > %d max buffer size", length, max_size);
    xSemaphoreGive(lk);
    return 0;
  }
  auto err = rf.readData(buf, length);
  std::string irq_status_str;
  auto status = rf.getIrqStatus();
  if (status & RADIOLIB_SX126X_IRQ_TIMEOUT) {
    irq_status_str += "t";
  }
  if (status & RADIOLIB_SX126X_IRQ_RX_DONE) {
    irq_status_str += "r";
  }
  if (status & RADIOLIB_SX126X_IRQ_CRC_ERR) {
    irq_status_str += "c";
  }
  if (status & RADIOLIB_SX126X_IRQ_HEADER_ERR) {
    irq_status_str += "h";
  }
  if (status & RADIOLIB_SX126X_IRQ_TX_DONE) {
    irq_status_str += "x";
  }
  if (!irq_status_str.empty()) {
    ESP_LOGI(TAG, "flag=%s", irq_status_str.c_str());
  }
  if (err != RADIOLIB_ERR_NONE) {
    ESP_LOGE(TAG, "failed to read data, code %d", err);
    xSemaphoreGive(lk);
    return 0;
  }
  xSemaphoreGive(lk);
  return length;
}

/**
 * @brief send status request repeatedly
 */
class StatusRequester {
public:
  static constexpr auto TAG                     = "StatusRequester";
  static constexpr auto NORMAL_REQUEST_INTERVAL = std::chrono::seconds{10};
  static constexpr auto FAST_REQUEST_INTERVAL   = std::chrono::seconds{5};
  /**
   * @brief the callback to check if the device map is empty
   */
  std::function<bool()> is_map_empty = []() { return true; };
  /**
   * @brief the callback to send the status request
   */
  std::function<void()> send_status_request = []() {};

private:
  TimerHandle_t status_request_timer         = nullptr;
  std::chrono::seconds last_request_interval = FAST_REQUEST_INTERVAL;

  [[nodiscard]] std::chrono::seconds get_target_request_interval() const {
    if (is_map_empty()) {
      return FAST_REQUEST_INTERVAL;
    } else {
      return NORMAL_REQUEST_INTERVAL;
    }
  }

public:
  void start() {
    const auto run_timer_task = [](TimerHandle_t timer) {
      auto &self = *static_cast<StatusRequester *>(pvTimerGetTimerID(timer));
      self.send_status_request();
      const auto target_interval = std::chrono::duration_cast<std::chrono::seconds>(self.get_target_request_interval());
      const auto target_millis   = std::chrono::duration_cast<std::chrono::milliseconds>(target_interval);
      if (self.last_request_interval != target_interval) {
        ESP_LOGI(TAG, "change request interval to %f second", static_cast<float>(target_millis.count()) / 1000.f);
        xTimerChangePeriod(timer, pdMS_TO_TICKS(target_millis.count()), portMAX_DELAY);
        self.last_request_interval = target_interval;
      }
      // https://www.freertos.org/FreeRTOS-timers-xTimerReset.html
      // https://greenwaves-technologies.com/manuals/BUILD/FREERTOS/html/timers_8h.html
      // https://cloud.tencent.com/developer/article/1914701
      xTimerReset(timer, portMAX_DELAY);
    };

    send_status_request();

    const auto target_interval = get_target_request_interval();
    const auto target_millis   = std::chrono::duration_cast<std::chrono::milliseconds>(target_interval);
    status_request_timer       = xTimerCreate("reqt",
                                              target_millis.count(),
                                              pdFALSE,
                                              this,
                                              run_timer_task);
    xTimerStart(status_request_timer, portMAX_DELAY);
  }
};

using namespace common;
using namespace common::lanely;

extern "C" void app_main();

void app_main() {
  constexpr auto TAG = "main";
  initArduino();
  pinMode(22,OUTPUT);
  pinMode(21,OUTPUT);
  Serial.begin(115200);
  Preferences pref;
  pref.begin(PREF_RECORD_NAME, true);
  const auto line_length_d   = pref.getFloat(PREF_LINE_LENGTH_NAME, DEFAULT_LINE_LENGTH.count());
  const auto active_length_d = pref.getFloat(PREF_ACTIVE_LENGTH_NAME, DEFAULT_ACTIVE_LENGTH.count());
  const auto line_LEDs_num_d = pref.getULong(PREF_LINE_LEDs_NUM_NAME, DEFAULT_LINE_LEDs_NUM);
  const auto total_length  = pref.getFloat(PREF_TOTAL_LENGTH_NAME, DEFAULT_TARGET_LENGTH.count());
  const auto color_d= pref.getULong(PREF_COLOR_NAME, utils::Colors::Red);
  const auto finish_time_d= pref.getULong(PREF_FTIME_NAME, utils::Colors::Red);
  const auto default_cfg   = ::lane::LaneConfig{
        .color         = color_d,
        .line_length   = lane::meter(line_len),//common::lanely::meter(line_length)
        .active_length = lane::meter(1.5),//lane::meter(active_length)
        .finish_length = lane::meter(total_length),
        .line_LEDs_num = line_len*10,//49*10+5
        .fps           = DEFAULT_FPS,
        .finish_time   = finish_time_d,
        .head_offset = 0,//65
  };
  const auto turn_d = pref.getFloat(PREF_TURN_TIME_NAME,2.0);
  const auto mode_d = pref.getUChar(PREF_PACE_MODE_NAME,0);
  const auto acce_d = pref.getFloat(PREF_ACCL_NAME,0.5);
  const auto default_pace = ::lane::LanePace{
        .pace_num = 1,
        .pace_time = {33,0,0,0,0},
        .else_deal  = 0,
        .platform_surface_time = 0,
        .platform_surface_range = 0,
        .acceleration = acce_d,
        .turn_time = turn_d ,
        .start_time = 0,
        .pace_mode  =mode_d,
  };
  pref.end();
  
    /********* lane initialization *********/
  constexpr auto lane_task = [](void *param) {
    auto &lane = *static_cast<lane::Lane *>(param);
    lane.loop();
    ESP_LOGE("lane", "lane loop exited");
  };
  auto s           = strip::AdafruitPixel(default_cfg.line_LEDs_num+default_cfg.head_offset, pin::LED, common::lanely::PIXEL_TYPE);
  static auto lane = lane::Lane{std::make_unique<decltype(s)>(std::move(s))};
  /********* end of lane initialization *********/


  /********* BLE initialization *********/
  NimBLEDevice::init(BLE_NAME);
  auto &server = *NimBLEDevice::createServer();
  server.setCallbacks(new ServerCallbacks());

  lane.initBLE(server);
  lane.setConfig(default_cfg);
  lane.setPace(default_pace);
  lane.oled_init(default_pace.pace_mode);

  ESP_ERROR_CHECK(lane.begin());

  //
  auto &ad = *NimBLEDevice::getAdvertising();
  ad.setName(BLE_NAME);
  ad.setScanResponse(false);

  // status_requester.start();

#ifdef DEBUG_SPEED
  float test_pace[10] = {0.1,0.05,0.15,0.08,0.12,0.06,0.14,0.1,0.03,0.17};
  lane.pace.acceleration = 2;
  lane.pace.turn_time = 3;
  lane.pace.pace_num = 10;
  memcpy(lane.pace.pace_time_pct,test_pace,lane.pace.pace_num);
  lane.pace.start_time = millis();
#endif

  auto scan_key_task = [] (void *p){
    while(true){
      lane.scan_key(key_down);
      lane.scan_key(key_up);
      vTaskDelay(5/portTICK_PERIOD_MS);
    }
  };
  // higher priority
  xTaskCreate(scan_key_task, "scan_key", 4096, nullptr
    ,6, nullptr);

  xTaskCreate(lane_task,
              "lane", 8192,
              &lane, 1,
              nullptr);
  server.start();

  NimBLEDevice::startAdvertising();
  ESP_LOGI(TAG, "Initiated");
  vTaskDelete(nullptr);
}
