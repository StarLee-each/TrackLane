#include <Arduino.h>
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
#include "utils.h"
#include "Lane.h"
#include "whitelist.h"
#include "common.h"
#include "ScanCallback.h"
#include "hr_lora.h"
#include "ble_hr_data.h"

// #define DEBUG_SPEED

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

  Preferences pref;
  pref.begin(PREF_RECORD_NAME, true);
  const auto line_length   = pref.getFloat(PREF_LINE_LENGTH_NAME, DEFAULT_LINE_LENGTH.count());
  const auto active_length = pref.getFloat(PREF_ACTIVE_LENGTH_NAME, DEFAULT_ACTIVE_LENGTH.count());
  const auto line_LEDs_num = pref.getULong(PREF_LINE_LEDs_NUM_NAME, DEFAULT_LINE_LEDs_NUM);
  const auto total_length  = pref.getFloat(PREF_TOTAL_LENGTH_NAME, DEFAULT_TARGET_LENGTH.count());
  const auto color         = pref.getULong(PREF_COLOR_NAME, utils::Colors::Red);
  const auto default_cfg   = ::lane::LaneConfig{
        .color         = color,
        .line_length   = lane::meter(line_length),
        .active_length = lane::meter(active_length),
        .finish_length = lane::meter(total_length),
        .line_LEDs_num = line_LEDs_num,
        .fps           = DEFAULT_FPS,
  };
  pref.end();

  static auto hal    = EspHal(pin::SCK, pin::MISO, pin::MOSI);
  static auto module = Module(&hal, pin::NSS, pin::DIO1, pin::LoRa_RST, pin::BUSY);
  auto *rf_lock      = xSemaphoreCreateMutex();
  if (rf_lock == nullptr) {
    ESP_LOGE("rf", "failed to create rf_lock");
    //esp_restart();
  }
  static auto rf = LLCC68(&module);
  const auto st        = rf.begin(433.2, 500.0, 10, 7,
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8, 1.6);
  if (st != RADIOLIB_ERR_NONE) {
    ESP_LOGE("rf", "failed, code %d", st);
  //  esp_restart();
  }

/********* recv interrupt initialization *********/
  auto evt_grp    = xEventGroupCreate();
  rf_receive_data = new rf_receive_data_t{.evt_grp = evt_grp};
  rf.setPacketReceivedAction([]() {
    const auto *data      = static_cast<rf_receive_data_t *>(rf_receive_data);
    BaseType_t task_woken = pdFALSE;
    if (data->evt_grp != nullptr) {
      const auto xResult = xEventGroupSetBitsFromISR(data->evt_grp, RecvEvt, &task_woken);
      if (xResult != pdFAIL) {
        portYIELD_FROM_ISR(task_woken);
      }
    }
  });
/********* end of recv interrupt initialization *********/

/********* status requester **********/
  static auto device_map        = device_name_map_t{};
  static auto status_requester  = StatusRequester{};
  status_requester.is_map_empty = []() {
    return device_map.empty();
  };
  auto send_status_request = [rf_lock]() {
    const auto req = HrLoRa::query_device_by_mac::t{
        .addr = HrLoRa::query_device_by_mac::broadcast_addr};

    uint8_t buf[16];
    const auto sz = HrLoRa::query_device_by_mac::marshal(req, buf, sizeof(buf));
    if (sz == 0) {
      ESP_LOGE("send status request", "failed to marshal");
      return;
    }
    try_transmit(buf, sz, rf_lock, send_lk_timeout_tick, rf);
  };
  status_requester.send_status_request = send_status_request;

  /********* recv task initialization            *********/
  static handle_message_callbacks_t handle_message_callbacks{};
  auto recv_task = [evt_grp, rf_lock](LLCC68 &rf) {
    constexpr auto TAG = "recv";
    for (;;) {
      xEventGroupWaitBits(evt_grp, RecvEvt, pdTRUE, pdFALSE, portMAX_DELAY);
      uint8_t data[255];
      const auto size = try_receive(data, sizeof(data), rf_lock, portMAX_DELAY, rf);
      if (size == 0) {
        continue;
      } else {
        ESP_LOGI(TAG, "data=%s(%d)", utils::toHex(data, size).c_str(), size);
      }
      // TODO: handle message stuff
      handle_message(data, size, handle_message_callbacks);
    }
  };

  struct recv_task_param_t {
    std::function<void(LLCC68 &)> task;
    LLCC68 *rf;
    TaskHandle_t handle;
    EventGroupHandle_t evt_grp;
  };

  /**
   * a helper function to run a function on a new FreeRTOS task
   */
  auto run_recv_task = [](void *pvParameter) {
    const auto param = static_cast<recv_task_param_t *>(pvParameter);
    [[likely]] if (param->task != nullptr && param->rf != nullptr) {
      param->task(*param->rf);
    } else {
      ESP_LOGW("recv task", "bad precondition");
    }
    const auto handle = param->handle;
    delete param;
    vTaskDelete(handle);
  };
  
  // static auto recv_param = recv_task_param_t{recv_task, &rf, nullptr, evt_grp};
  // xTaskCreate(run_recv_task,
  //             "recv", 4096,
  //             &recv_param, 1,
  //             &recv_param.handle);
  // /********** end of recv task initialization **********/

  // ESP_LOGI(TAG, "LoRa RF initiated");

  /********* lane initialization *********/
  constexpr auto lane_task = [](void *param) {
    auto &lane = *static_cast<lane::Lane *>(param);
    lane.loop();
    ESP_LOGE("lane", "lane loop exited");
  };
  auto s           = strip::AdafruitPixel(default_cfg.line_LEDs_num, pin::LED, common::lanely::PIXEL_TYPE);
  static auto lane = lane::Lane{std::make_unique<decltype(s)>(std::move(s))};
  /********* end of lane initialization *********/

  /********* BLE initialization *********/
  NimBLEDevice::init(BLE_NAME);
  auto &server = *NimBLEDevice::createServer();
  server.setCallbacks(new ServerCallbacks());

  lane.initBLE(server);
  lane.setConfig(default_cfg);
  ESP_ERROR_CHECK(lane.begin());

  auto &hr_service = *server.createService(BLE_CHAR_HR_SERVICE_UUID);
  auto &hr_char    = *hr_service.createCharacteristic(BLE_CHAR_HEARTBEAT_UUID,
                                                      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  hr_service.start();

  handle_message_callbacks = handle_message_callbacks_t{
      .get_device_by_key = [](int key) -> std::optional<HrLoRa::hr_device::t> {
        auto it = device_map.find(key);
        if (it == device_map.end()) {
          return std::nullopt;
        } else {
          if (it->second.device.has_value()) {
            return std::make_optional(it->second.device.value());
          } else {
            return std::nullopt;
          }
        }
      },
      .update_device = [](repeater_t repeater) {
        constexpr auto TAG = "update_device";
        if (!repeater.device.has_value()){
          ESP_LOGW(TAG, "null device");
          return true;
        }
        auto repeater_addr = repeater.repeater_addr;
        // search for the repeater's addr first
        const auto addr_it = std::find_if(device_map.begin(), device_map.end(),
                                    [&repeater_addr](const auto &pair) {
                                      const auto [key, r] = pair;
                                      return std::equal(r.repeater_addr.begin(),
                                                        r.repeater_addr.end(),
                                                        repeater_addr.begin());
                                    });

        if (addr_it == device_map.end()) {
          // goto key_it since the repeater's addr is not in the map
        } else {
          // check if the key of the repeater is changed
          if (repeater.key == addr_it->second.key) {
            // same key, just update
            addr_it->second = std::move(repeater);
            return true;
          } else {
            // key mismatch, remove the old one
            const auto &addr = addr_it->second.repeater_addr;
            ESP_LOGI(TAG, "%s key %d (new) != %d (old)",
                     utils::toHex(addr.data(),addr.size()).c_str(),
                     addr_it->second.key, repeater.key);
            device_map.erase(addr_it);
          }
        }

        const auto key_it = device_map.find(repeater.key);
        if (key_it == device_map.end()) {
          if (device_map.size() >= MAX_DEVICE_COUNT) {
            ESP_LOGW(TAG, "full device map; clear it");
            device_map.clear();
          }
          const auto& addr = repeater.repeater_addr;
          const auto& dev_addr = repeater.device->addr;
          ESP_LOGI(TAG, "new repeater addr=%s; key=%d; dev_addr=%s; dev_name=%s;",
                   utils::toHex(addr.data(), addr.size()).c_str(),
                   repeater.key,
                   utils::toHex(dev_addr.data(), dev_addr.size()).c_str(),
                   repeater.device->name.c_str());
          device_map.insert({repeater.key, std::move(repeater)});
          return true;
        } else {
          auto &addr = key_it->second.repeater_addr;
          ESP_LOGW(TAG, "key %d is already used by %s", repeater.key,
                   utils::toHex(addr.data(), addr.size()).c_str());
          return false;
        } },
      .on_hr_data    = [&hr_char](std::string name, int hr) {
        constexpr auto TAG = "on_hr_data";
        ESP_LOGI(TAG, "hr=%d; name=%s", hr, name.c_str());
        const auto ble_hr_data = ble::hr_data::t{
            .name = std::move(name),
            .hr = static_cast<uint8_t>(hr)
        };
        uint8_t buf[32] = {0};
        const auto sz         = ble::hr_data::marshal(ble_hr_data, buf, sizeof(buf));
        if (sz == 0) {
          ESP_LOGE(TAG, "failed to marshal");
          return;
        }
        hr_char.setValue(buf, sz);
        hr_char.notify(); },
      .rf_send       = [rf_lock](uint8_t *pdata, size_t size) { try_transmit(pdata, size, rf_lock, send_lk_timeout_tick, rf); },
  };

  auto &ad = *NimBLEDevice::getAdvertising();
  ad.setName(BLE_NAME);
  ad.setScanResponse(false);

  status_requester.start();

#ifdef DEBUG_SPEED
  lane.setStatus(lane::LaneStatus::FORWARD);
  lane.setSpeed(2);
#endif

  // higher priority
  xTaskCreate(lane_task,
              "lane", 8192,
              &lane, 5,
              nullptr);

  server.start();
  NimBLEDevice::startAdvertising();
  ESP_LOGI(TAG, "Initiated");
  vTaskDelete(nullptr);
}
