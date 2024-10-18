//
// Created by Kurosu Chan on 2022/7/13.
//

#include "Lane.h"
#include "NimBLECharacteristic.h"
#include "Strip.hpp"
#include "common.h"
#include "esp32-hal.h"
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
std::string statusToStr(LaneStatus status) {
  static const std::map<LaneStatus, std::string> LANE_STATUS_STR = {
      {LaneStatus::FORWARD, "FORWARD"},
      {LaneStatus::BACKWARD, "BACKWARD"},
      {LaneStatus::STOP, "STOP"},
  };
  return LANE_STATUS_STR.at(status);
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
  auto ret_input = input;
  auto ret = last_state;
  auto stop_case     = [=]() {
    switch (input.status) {
      case LaneStatus::FORWARD: {
        auto ret1   = zero_state;
        ret1.speed  = input.speed;
        ret1.status = LaneStatus::FORWARD;
        return ret1;
      }
      case LaneStatus::BACKWARD: {
        auto ret1   = zero_state;
        ret1.speed  = input.speed;
        ret1.status = LaneStatus::BACKWARD;
        return ret1;
      }
      default:
        return zero_state;
    }
  };
  
  if(input.pause_tick > 0){
    if((millis() - input.pause_tick) > (pace_in.turn_time*1000)){
      ret_input.pause_tick = 0;
      ret.status = revert_state(last_state.status);
    }else{
      auto ret1 = last_state;
      return {ret1, input};
    }
  }
  switch (last_state.status) {
    case LaneStatus::STOP:
    case LaneStatus::BLINK: {
      return {stop_case(), input};
    }
    default: {
      if (input.status == LaneStatus::STOP ||
          input.status == LaneStatus::BLINK) {
        return {zero_state, input};
      }
      if (input.status != last_state.status) {
        ESP_LOGW(TAG, "Invalid status changed from %s to %s", statusToStr(last_state.status).c_str(), statusToStr(input.status).c_str());
        auto param   = input;
        param.status = last_state.status;
        return {last_state, param};
      }
      ret.speed = input.speed;
      // I assume every time call this function the time interval is 1/fps
      ret.shift      = last_state.shift + meter(ret.speed / cfg.fps);
      auto temp_head = last_state._head + meter(ret.speed / cfg.fps);
      auto err       = meter(ret.speed / cfg.fps);
      if (temp_head >= (cfg.active_length + cfg.line_length - err)) {
        //ret.status = revert_state(last_state.status);
        ret_input.pause_tick = millis();
        ret.head   = meter(0);
        ret._head  = ret.head;
        ret.tail   = meter(0);
      } else if (temp_head >= cfg.line_length) {
        ret._head = temp_head;
        ret.head  = cfg.line_length;
        auto t    = temp_head - cfg.active_length;
        ret.tail  = t > cfg.line_length ? cfg.line_length : t;
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

[[noreturn]] void Lane::loop() {
  auto instant                  = Instant();
  auto constexpr DEBUG_INTERVAL = std::chrono::seconds(1);
  ESP_LOGI(TAG, "loop");
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
        ESP_LOGI(TAG, "head=%.2f; tail=%.2f; shift=%.2f; speed=%.2f; status=%s; color=%0x06x; fps=%f",
                 state.head.count(), state.tail.count(), state.shift.count(), state.speed,
                 statusToStr(state.status).c_str(), cfg.color, this->cfg.fps);
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
        delete_timer();
        stop();
        vTaskDelay(delay);
        strip->fill_and_show_forward(0, cfg.line_LEDs_num, cfg.color);
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
float Lane::make_contain(float in, float standard, float offset){
  if(in > standard){
    in -= offset;
    if(in < standard){
      in = standard;
      return in;
    }
  }else if(in < standard){
    in += offset;
    if(in > standard){
      in = standard;
    }
  }else{
    in = standard;
    return in;
  }
  return in;
}
void Lane::update_speed(const LaneState &last_state, const LaneConfig &cfg, const LaneParams &input , const LanePace &pace){
  const auto shift = last_state.shift.count();
  const auto finish_length = cfg.finish_length.count();
  auto cal_speed = input.speed;
  uint8_t stage = shift*pace.pace_num*1.0/finish_length;
  //const uint8_t turn_times = finish_length / 50.0 -1;
  if(stage>9){
    stage = 9;
  }
  auto stage_finish_time = 0.0;
  for(int i=0; i< stage; i++){
    stage_finish_time += cfg.finish_time * pace.pace_time_pct[i];
  }
  const auto stage_left_length = finish_length * (stage + 1.0) / pace.pace_num - shift;
  const auto stage_left_time = stage_finish_time + pace.start_time - millis();
  if(stage_left_time <= 0){
    return;
  }
  cal_speed = stage_left_length / stage_left_time;
  cal_speed = this->make_contain(input.speed,cal_speed, pace.acceleration);
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
  this->update_speed(this->state, this->cfg, this->params,this->pace);
  auto [next_state, params] = nextState(this->state, this->cfg, this->params, this->pace);
  // meter
  const auto head       = this->state.head.count();
  const auto tail       = this->state.tail.count();
  const auto length     = head - tail >= 0 ? head - tail : 0;
  const auto tail_index = meterToLEDsCount(tail, LEDsPerMeter());
  const auto count      = meterToLEDsCount(length, LEDsPerMeter());
  this->params          = params;
  this->state           = next_state;
  switch (next_state.status) {
    case LaneStatus::FORWARD: {
      strip->fill_and_show_forward(tail_index, count, cfg.color);
      break;
    }
    case LaneStatus::BACKWARD: {
      strip->fill_and_show_backward(tail_index, count, cfg.color);
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
                                                  NIMBLE_PROPERTY::READ |  NIMBLE_PROPERTY::WRITE);

  ble.service->start();
  
}
}
