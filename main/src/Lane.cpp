//
// Created by Kurosu Chan on 2022/7/13.
//

#include "Lane.h"
#include "NimBLECharacteristic.h"
#include "Strip.hpp"
#include "common.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal.h"
//#include "../../../../../esp-idf/components/wpa_supplicant/src/utils/common.h"

#include <cstdint>
#include <esp_check.h>

static const auto TAG = "lane";

// the resolution is the clock frequency instead of strip frequency
constexpr auto LED_STRIP_RMT_RES_HZ = (10 * 1000 * 1000); // 10MHz

static inline int meterToLEDsCount(const float meter, const float LEDs_per_meter) {
  return std::abs(static_cast<int>(std::round(meter * LEDs_per_meter))) + 1;
}

static inline float LEDsCountToMeter(const uint32_t count, const float LEDs_per_meter) {
  if (count <= 1) {
    return 0;
  } else {
    return static_cast<float>(count - 1) / LEDs_per_meter;
  }
}

namespace lane {
static constexpr auto TIMER_TIMEOUT_TICKS = 100;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, 32, 22, 21);
std::string statusToStr(LaneStatus status) {
  switch (status) {
    case LaneStatus::FORWARD: return "Forward";
    case LaneStatus::BACKWARD: return "Backward";
    case LaneStatus::STOP: return "Stop";
    case LaneStatus::BLINK: return "Blink";
    default: return "Unknown";
  }
}

inline LaneStatus revert_state(LaneStatus state) {
  if (state == LaneStatus::STOP) {
    return LaneStatus::STOP;
  }
  if (state == LaneStatus::FORWARD) {
    return LaneStatus::BACKWARD;
  } else {
    return LaneStatus::FORWARD;
  }
}
/**
 * @brief get the next state. should be a pure function.
 * @param [in]last_state
 * @param [in]cfg
 * @param [in]input param
 * @return the next state and the param (external input/state)
 */
static std::tuple<LaneState, LaneParams>
nextState(const LaneState &last_state, const LaneConfig &cfg, const LaneParams &input, const LanePace &pace_in) {
  constexpr auto TAG = "lane::nextState";
  auto zero_state    = LaneState::zero();
  auto ret_input     = input;
  auto ret           = last_state;
  auto stop_case     = [=]() {
    switch (ret_input.status) {
      case LaneStatus::FORWARD: {
        auto ret1   = zero_state;
        ret1.speed  = ret_input.speed;
        ret1.status = LaneStatus::FORWARD;
        return ret1;
      }
      case LaneStatus::BACKWARD: {
        auto ret1   = zero_state;
        ret1.speed  = ret_input.speed;
        ret1.status = LaneStatus::BACKWARD;
        return ret1;
      }
      default:
        return zero_state;
    }
  };
  if (ret.shift >= cfg.finish_length and ret_input.act_mode == ActMode::TIME_RELY) {
    // ret = zero_state;
    ret.status = LaneStatus::BLINK;
    ret_input.status = LaneStatus::BLINK;
    return {ret, ret_input};
  }
  if (ret_input.pause_tick > 0) {
    if ((millis()- ret_input.pause_tick) > (pace_in.turn_time * 1000-150)) {
      ret_input.pause_tick = 0;
      ret.status           = revert_state(last_state.status);
    } else {
      //ESP_LOGI("LANE", "Paused turn_time=%.1f",pace_in.turn_time);
      auto ret1 = last_state;
      return {ret1, ret_input};
    }
  }

  switch (ret.status) {
    case LaneStatus::STOP:
    case LaneStatus::BLINK: {
      return {stop_case(), ret_input};
    }
    default: {
      if (input.status == LaneStatus::STOP ||
          input.status == LaneStatus::BLINK) {
        return {zero_state, input};
          }
      // if (input.status != last_state.status) {
      //   ESP_LOGW(TAG, "Invalid status changed from %s to %s", statusToStr(last_state.status).c_str(), statusToStr(input.status).c_str());
      //   auto param   = input;
      //   param.status = last_state.status;
      //   return {last_state, param};
      // }
      ret.speed = ret_input.speed;
      // I assume every time call this function the time interval is 1/fps
      ret.shift      = last_state.shift + meter(ret_input.speed / cfg.fps);
      auto temp_head = meter((static_cast<int>(ret.shift.count()*10000)%static_cast<int>(cfg.line_length.count()*10000))/10000.0);
      auto err       = std::floor(ret_input.speed *1000 / cfg.fps);


      if (temp_head < meter(err/1000.0) and ret.shift < cfg.finish_length and ret.shift > meter(1)) {    //(cfg.line_length-meter(ret.speed))
        // ret.status = revert_state(last_state.status);
        ret_input.pause_tick = millis();
        ret.head  = cfg.line_length + temp_head;
        ret.tail  = ret.head - cfg.active_length;
        ESP_LOGI("LANE","call pause head=%.5f err = %.6f",ret.head.count() , err/1000.0);
        // ret.head             = meter(0);
        // ret._head            = ret.head;
        // ret.tail             = meter(0);
        // } else if (temp_head >= cfg.line_length) {
        //   ret._head = temp_head;
        //   ret.head  = cfg.line_length;
        //   auto t    = temp_head - cfg.active_length;
        //   ret.tail  = t > cfg.line_length ? cfg.line_length : t;
      } else {
        ret.head       = temp_head;
        ret._head      = temp_head;
        auto temp_tail = temp_head - cfg.active_length;
        ret.tail       = temp_tail > meter(0) ? temp_tail : meter(0);
      }
      return {ret, ret_input};
    }
  }
}

void Lane::stop() const {
  if (strip == nullptr) {
    ESP_LOGE(TAG, "strip is null");
    return;
  }
  strip->clear();
  strip->show();
}

struct UpdateTaskParam {
  std::function<void()> *fn;
  TaskHandle_t handle;
  ~UpdateTaskParam() {
    delete fn;
    fn = nullptr;
    if (handle != nullptr) {
      vTaskDelete(handle);
    }
  }
};
/**
 * key ctrl task
 */
void Lane::scan_key(uint8_t key) {
  if (digitalRead(key) == 0 and key_char.push_tick == 0 and key_char.who == 0) {
    key_char.push_tick = millis();
    key_char.who       = key;
    if (key_char.click == key_char.non) {
      key_char.act_tick = millis();
    }
  } else if (key == key_char.who and digitalRead(key) == 1 and 10<(millis() - key_char.push_tick) and (millis() - key_char.push_tick)<500 and key_char.click == key_char.non) {
    key_char.click = key_char.single_click;
  } else if (key == key_char.who and digitalRead(key) == 1 and key_char.push_tick > 500 and key_char.click == key_char.non) {
    key_char.click = key_char.long_click;
  } else if (key == key_char.who and key_char.click == key_char.single_click and digitalRead(key) == 1 and (millis() - key_char.push_tick > 10)) {
    key_char.click = key_char.double_click; // nerver in, needs a single-click delay to work
  } else if (key == key_char.who and digitalRead(key) == 1) {
    key_char = key_char_zero;
  }
  switch (key_char.click) {
    case key_char.non:
      break;
    case key_char.single_click:
      this->single_click_f(key);
      break;
    case key_char.double_click:
      this->double_click_f(key);
      break;
    case key_char.long_click:
      this->long_click_f(key);
      break;
  }
}
[[noreturn]] void Lane::loop() {
  auto instant                  = Instant();
  auto constexpr DEBUG_INTERVAL = std::chrono::seconds(1);
  ESP_LOGI(TAG, "loop");
  int p =0;

  // for( p=0; p<11; p++) {
  //
  //   strip->fill_and_show_forward(p*10,15,0x00ff00);
  //   if(p>9) {
  //     p=0;
  //   }
  //   ESP_LOGI("led test:","tail = %d",p);
  // vTaskDelay(1000 / portTICK_PERIOD_MS);
  // }
  for (;;) {
    if (strip == nullptr) {
      ESP_LOGE(TAG, "null strip");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // TODO: use xTimerCreateStatic to avoid heap allocation repeatedly
    auto delete_timer = [this]() {
      if (this->timer_handle != nullptr) {
        ESP_LOGI(TAG, "delete timer");
        auto ok = xTimerStop(this->timer_handle, TIMER_TIMEOUT_TICKS);
        if (!ok) {
          ESP_LOGE(TAG, "Failed to stop timer");
          return;
        }
        ok = xTimerDelete(this->timer_handle, TIMER_TIMEOUT_TICKS);
        if (!ok) {
          ESP_LOGE(TAG, "Failed to delete timer");
          return;
        }
        this->timer_handle = nullptr;
      } else {
      }
    };
    /**
     * @brief try to create a timer if there is no timer running.
     */
    auto try_create_timer = [this]() {
      // https://www.nextptr.com/tutorial/ta1430524603/capture-this-in-lambda-expression-timeline-of-change
      auto notify_fn = [this]() {
        ESP_LOGI(TAG, "line_len=%.1f head=%.2f; tail=%.2f; shift=%.2f; speed=%.2f; status=%s; color=%0x; fps=%.1f \r\nacce=%.1f; start_time=%d; turn_time=%.2f",
                 cfg.line_length.count(), state.head.count(), state.tail.count(), state.shift.count(), state.speed,
                 statusToStr(state.status).c_str(), cfg.color, this->cfg.fps, this->pace.acceleration, this->pace.start_time, this->pace.turn_time);
        this->notifyState(this->state);
        xTimerReset(this->timer_handle, TIMER_TIMEOUT_TICKS);
      };

      auto run_notify_fn = [](TimerHandle_t handle) {
        const auto &param = *static_cast<notify_timer_param *>(pvTimerGetTimerID(handle));
        param.fn();
      };

      // otherwise a timer is running
      if (this->timer_handle == nullptr) {
        this->timer_param.fn = notify_fn;
        this->timer_handle   = xTimerCreate("notify_timer",
                                            pdMS_TO_TICKS(common::lanely::BLUE_TRANSMIT_INTERVAL.count()),
                                            pdFALSE,
                                            &this->timer_param,
                                            run_notify_fn);
        [[unlikely]] if (this->timer_handle == nullptr) {
          ESP_LOGE(TAG, "Failed to create timer");
          abort();
        }
        xTimerStart(this->timer_handle, TIMER_TIMEOUT_TICKS);
      }
    };

    switch (params.status) {
      case LaneStatus::FORWARD:
      case LaneStatus::BACKWARD: {
        instant.reset();
        iterate();
        try_create_timer();
        const auto diff  = std::chrono::duration_cast<std::chrono::milliseconds>(instant.elapsed());
        const auto delay = std::chrono::milliseconds(static_cast<uint16_t>(1000 / cfg.fps)) - diff;
        if (delay < std::chrono::milliseconds(0)) [[unlikely]] {
          ESP_LOGW(TAG, "timeout %lld", delay.count());
        } else {
          const auto ticks = pdMS_TO_TICKS(delay.count());
          vTaskDelay(ticks);
        }
        break;
      }
      case LaneStatus::STOP: {
        this->state = LaneState::zero();
        delete_timer();
        stop();
        vTaskDelay(pdMS_TO_TICKS(common::lanely::HALT_INTERVAL.count()));
        break;
      }
      case LaneStatus::BLINK: {
        constexpr auto BLINK_INTERVAL = std::chrono::milliseconds(500);
        constexpr auto delay          = pdMS_TO_TICKS(BLINK_INTERVAL.count());
        //delete_timer();
        stop();
        vTaskDelay(delay);
        const auto tail_index = meterToLEDsCount(this->state.tail.count(), LEDsPerMeter());
        const auto count      = meterToLEDsCount(this->cfg.active_length.count(), LEDsPerMeter());
        const int finish_len = static_cast<int>(this->cfg.finish_length.count());
        const size_t start = this->state.head.count()*10;
        if((finish_len/50)%2 == 0) {
          strip->fill_and_show_forward(count, count, cfg.color, cfg.head_offset);
        }else {
          strip->fill_and_show_backward(count, count, cfg.color, cfg.head_offset, cfg.line_LEDs_num);
        }
        vTaskDelay(delay);
        break;
      }
      default:
        break;
    }
  }
}

void Lane::setMaxLEDs(uint32_t new_max_LEDs) {
  if (strip == nullptr) {
    ESP_LOGE(TAG, "strip is null");
    return;
  }
  strip->set_max_LEDs(new_max_LEDs);
}

/**
 * @brief initialize the strip. this function should only be called once.
 * @return StripError::OK if the strip is not inited, otherwise StripError::HAS_INITIALIZED.
 */
esp_err_t Lane::begin() {
  if (strip == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  strip->begin();
  return ESP_OK;
}

void Lane::notifyState(LaneState st) {
  const auto TAG = "Lane::notifyState";
  if (this->ble.ctrl_char == nullptr) {
    ESP_LOGE(TAG, "BLE not initialized");
    return;
  }
  auto &notify_char = *this->ble.ctrl_char;
  auto buf          = std::array<uint8_t, LaneState_size>();
  ::LaneState pb_st = LaneState_init_zero;
  pb_st.head        = st.head.count();
  pb_st.tail        = st.tail.count();
  pb_st.shift       = st.shift.count();
  pb_st.speed       = st.speed;
  pb_st.status      = static_cast<::LaneStatus>(st.status);
  auto stream       = pb_ostream_from_buffer(buf.data(), buf.size());
  if (const auto ok = pb_encode(&stream, LaneState_fields, &pb_st); !ok) {
    ESP_LOGE(TAG, "Failed to encode the state");
    return;
  }
  auto h = utils::toHex(buf.cbegin(), stream.bytes_written);
  notify_char.setValue(buf.cbegin(), stream.bytes_written);
  notify_char.notify();
}

meter Lane::lengthPerLED() const {
  auto l = this->cfg.line_length;
  auto n = this->cfg.line_LEDs_num;
  return l / n;
}

float Lane::LEDsPerMeter() const {
  auto l = this->cfg.line_length.count();
  auto n = this->cfg.line_LEDs_num;
  return n / l;
}
float Lane::make_contain(float in, float standard, float offset) {
  if (in > standard) {
    in -= offset;
    if (in < standard) {
      in = standard;
      return in;
    }
  } else if (in < standard) {
    in += offset;
    if (in > standard) {
      in = standard;
    }
  } else {
    in = standard;
    return in;
  }
  return in;
}
void Lane::update_speed(const LaneState &last_state, const lane::LaneConfig &cfg, const LaneParams &input, const LanePace &pace) {
  if(last_state.status == LaneStatus::BLINK || last_state.status == LaneStatus::STOP) {
    return;
  }

  const auto shift         = last_state.shift.count();
  const auto finish_length = cfg.finish_length.count();
  auto cal_speed           = input.speed;
  auto pace_num   = pace.pace_num;
  if(shift == 0) {
    setStartTime();
  }
  if(pace.platform_surface_time > 0 and shift < pace.platform_surface_range){
    cal_speed = pace.platform_surface_range / pace.platform_surface_time;
    this->setSpeed(cal_speed);
    return;
  }
  if(pace_num <= 1 ){
    pace_num = 1;
    this->setPaceTime0(cfg.finish_time);
  }
  uint8_t stage            = shift * pace_num * 1.0 / finish_length;
  // const uint8_t turn_times = finish_length / 50.0 -1;
  if (stage > 15) {
    stage = 15;
  }
  auto stage_finish_time = 0.0;
  auto named_stage =5;
  auto named_time = 0.0;
//  for (int i = 0; i <= stage; i++) {    //pace by stage_percent
//    stage_finish_time += cfg.finish_time *1000.0* pace.pace_time[i];//ms
//  }
  for(uint8_t i =0;i<5;i++){
    named_time += pace.pace_time[i];
    if(pace.pace_time[i] == 0){
      named_stage = i;
      break;
    }
  }
  if(stage < named_stage){
    stage_finish_time = pace.pace_time[stage]*1000.0;
  }else{
    auto stage_finish_time_avg = (cfg.finish_time - named_time)/(pace_num - named_stage);
    if(pace.else_deal == 0){
      stage_finish_time = stage_finish_time_avg;
    }else{
      stage_finish_time = (stage_finish_time_avg - 0.5 * pace.else_deal * (pace_num - named_stage) ) + (stage +1 -named_stage)*pace.else_deal;
    }
  }
  const float stage_left_length = finish_length * (stage + 1.0) / pace_num - shift;//m
  const auto stage_left_time   = stage_finish_time - (millis() - pace.start_time);//ms

  if (stage_left_time <= 0) {
    cal_speed = input.speed +pace.acceleration / cfg.fps;
    ESP_LOGI("SPEED_CAL","left_notime");
    return;
  }else {
    cal_speed = stage_left_length *1000.0/ stage_left_time;
  }
  ESP_LOGI("SPEED CAL","IN stage : %d \r\nleft time: %.2fms\r\nstage needs time : %.2fms\r\n",
           stage,stage_left_time,stage_finish_time);
  if (input.speed == 0) {
    ;
  }else {
    cal_speed = this->make_contain(input.speed, cal_speed, pace.acceleration / cfg.fps);
  }
  this->setSpeed(cal_speed);
}
/**
 * @brief iterate the strip to the next state and set the corresponding LEDs.
 * @note this function will block the current task for a short time (for the strip to show the LEDs).
 *       To make the strip to run with certain FPS, please add a delay outside this function
 *       (minus the time used in this function).
 */
void Lane::iterate() {
  if (strip == nullptr) {
    ESP_LOGE(TAG, "strip is null");
    return;
  }
  switch(params.act_mode) {
    case ActMode::TIME_RELY:
      if(pace.start_time == 0 or (tick_buf != 0 and (millis() - tick_buf)>500)){
        ESP_LOGI("itrate","tick_buf = %d;;millis=%d;;start_time=%d",tick_buf,millis(),pace.start_time);
        tick_buf = millis();
        this->update_speed(this->state, this->cfg, this->params, this->pace);
      }
      break;
    case ActMode::SPEED_RELY:
      break;
  }

  auto [next_state, params] = nextState(this->state, this->cfg, this->params, this->pace);
  // meter
  this->params          = params;
  this->state           = next_state;
  const auto head       = this->state.head.count();
  const auto tail       = this->state.tail.count();
  const auto length     = head - tail >= 0 ? head - tail : 0;
  const auto head_index = meterToLEDsCount(head, LEDsPerMeter());
  // const auto count      = meterToLEDsCount(length, LEDsPerMeter());
  const auto count      = meterToLEDsCount(cfg.active_length.count(), LEDsPerMeter());
  switch (next_state.status) {
    case LaneStatus::FORWARD: {
      strip->fill_and_show_forward(head_index , count, cfg.color, cfg.head_offset);
      break;
    }
    case LaneStatus::BACKWARD: {
      strip->fill_and_show_backward(head_index , count, cfg.color, cfg.head_offset,cfg.line_LEDs_num);
      break;
    }
    default:
      break;
      ;
  }
}

void Lane::_initBLE(NimBLEServer &server, Lane::LaneBLE &ble) {
  ble.service = server.createService(common::BLE_SERVICE_UUID);

  ble.ctrl_char = ble.service->createCharacteristic(common::BLE_CHAR_CONTROL_UUID,
                                                    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  ble.ctrl_char->setCallbacks(&ble.ctrl_cb);

  /// write to control and read/notify for the state
  ble.config_char = ble.service->createCharacteristic(common::BLE_CHAR_CONFIG_UUID,
                                                      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  ble.config_char->setCallbacks(&ble.config_cb);

  ble.pace_char = ble.service->createCharacteristic(common::BLE_CHAR_PACE_UUID,
                                                    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  ble.pace_char->setCallbacks(&ble.pace_cb);

  ble.service->start();
}
}
