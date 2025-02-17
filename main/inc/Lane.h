//
// Created by Kurosu Chan on 2022/7/13.
//

#ifndef HELLO_WORLD_STRIP_H
#define HELLO_WORLD_STRIP_H

#include <cstddef>
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <memory>
#include <stdint.h>
#include "HardwareSerial.h"
#include "utils.h"
#include "lane.pb.h"
#include "Strip.hpp"
#include "common.h"
#include "U8g2lib.h"
#include <Wire.h>
#define key_down 16
#define key_up 17
#define line_len 20
namespace lane {
using centimeter = common::lanely::centimeter;
using meter      = common::lanely::meter;
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
struct notify_timer_param {
  std::function<void()> fn;
};

enum class LaneStatus {
  FORWARD  = ::LaneStatus_FORWARD,
  BACKWARD = ::LaneStatus_BACKWARD,
  STOP     = ::LaneStatus_STOP,
  BLINK    = ::LaneStatus_BLINK,
  KEEP     = ::LaneStatus_KEEP,
};

std::string statusToStr(LaneStatus status);

enum class ActMode {
  TIME_RELY =0,
  SPEED_RELY,
};

enum class LaneError {
  OK = 0,
  ERROR,
  HAS_INITIALIZED,
};

struct LaneState {
  // scalar cumulative distance shift
  meter shift = meter(0);
  // m/s
  float speed = 0;
  // head should be always larger than tail
  meter head = meter(0);
  // hidden head for calculation
  meter _head       = meter(0);
  meter tail        = meter(0);
  LaneStatus status = LaneStatus::STOP;
  static LaneState zero() {
    return LaneState{
        .shift  = meter(0),
        .speed  = 0,
        .head   = meter(0),
        ._head  = meter(0),
        .tail   = meter(0),
        .status = LaneStatus::STOP,
    };
  };
};

// won't change unless the status is STOP
struct LaneConfig {
  uint32_t color;
  /// @brief the length of the line
  meter line_length;
  /// @brief the length of the active part of the line
  meter active_length;
  /// @brief after this length, the line would be stop and reset
  meter finish_length;
  /// @brief the number of LEDs that form the line
  uint32_t line_LEDs_num;
  float fps;
  uint32_t finish_time;
  uint8_t head_offset;
};

/**
 * @brief input; external, outside world could change it
 * (instead of changing the state directly)
 */
struct LaneParams {
  float speed;
  LaneStatus status;
  int pause_tick;
  ActMode act_mode;
};

struct LanePace {
  uint8_t pace_num;
  float pace_time[5];
  float else_deal;
  float platform_surface_time;
  float platform_surface_range;
  float acceleration;
  float turn_time;
  int start_time;
  int pace_mode;
};

struct LaneKey {
  enum click_type{
    non =0,
    single_click=1,
    double_click,
    long_click,
  };
  uint8_t click;
  uint32_t push_tick;
  uint32_t act_tick;
  uint8_t who;
};
// note: I assume every time call this function the time interval is 1/fps
std::tuple<LaneState, LaneParams> static nextState(const LaneState &last_state, const LaneConfig &cfg, const LaneParams &input, const LanePace &pace_in);

//**************************************** Lane *********************************/

/**
 * @brief The Lane class
 */
class Lane {
  friend class ControlCharCallback;
  friend class ConfigCharCallback;
  friend class PaceCharCallback;
  
private:
  /**
   * @brief The ControlCharCallback class, which can notify the client the current state of the strip and accept the input from the client.
   */
  class ControlCharCallback final : public NimBLECharacteristicCallbacks {
    lane::Lane &lane;

  public:
    void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo) override;

    explicit ControlCharCallback(lane::Lane &lane) : lane(lane){};
  };

  class ConfigCharCallback final : public NimBLECharacteristicCallbacks {
    lane::Lane &lane;

  public:
    /// would expect LaneConfig variant
    void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo) override;
    /// would output LaneConfigRO variant
    void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;

    explicit ConfigCharCallback(lane::Lane &lane) : lane(lane) {}
  };
  /**
   * @brief The PaceCharCallback class, which can notify the client the current state of the strip and accept the input from the client.
   */
  class PaceCharCallback final : public NimBLECharacteristicCallbacks {
    lane::Lane &lane;

  public:
    void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo) override;

    explicit PaceCharCallback(lane::Lane &lane) : lane(lane){};
  };



  /**
   * @brief Bluetooth LE and lane related stuff
   */
  struct LaneBLE {
    Lane *lane                        = nullptr;
    NimBLECharacteristic *ctrl_char   = nullptr;
    NimBLECharacteristic *config_char = nullptr;
    NimBLECharacteristic *pace_char = nullptr;
    NimBLEService *service = nullptr;
    ControlCharCallback ctrl_cb;
    ConfigCharCallback config_cb;
    PaceCharCallback pace_cb;
    explicit LaneBLE(Lane *lane) : lane(lane), ctrl_cb(*lane), config_cb(*lane), pace_cb(*lane) {}
  };

  Preferences pref;
  using strip_ptr_t = std::unique_ptr<strip::IStrip>;
  strip_ptr_t strip = nullptr;
  notify_timer_param timer_param{[]() {}};
  TimerHandle_t timer_handle = nullptr;
  std::array<uint8_t, common::lanely::DECODE_BUFFER_SIZE>
      decode_buffer = {0};

  LaneBLE ble    = LaneBLE{this};
  LaneConfig cfg = {
      .color         = utils::Colors::Red,
      .line_length   = common::lanely::DEFAULT_LINE_LENGTH,
      .active_length = utils::length_cast<meter>(common::lanely::DEFAULT_ACTIVE_LENGTH),
      .finish_length = common::lanely::DEFAULT_TARGET_LENGTH,
      .line_LEDs_num = common::lanely::DEFAULT_LINE_LEDs_NUM,
      .fps           = common::lanely::DEFAULT_FPS,
      .finish_time  = common::lanely::DEFAULT_FINISH_TIME,
  };
  LaneState state   = LaneState::zero();
  LaneParams params = {
      .speed  = 0,
      .status = LaneStatus::STOP,
      .pause_tick = 0,
      .act_mode = ActMode::TIME_RELY,
  };
  LaneKey key_char ={
      .click = LaneKey::non,
      .push_tick =0,
      .act_tick  = 0,
      .who=0,
  };
  LaneKey key_char_zero ={
      .click = LaneKey::non,
      .push_tick =0,
      .act_tick  = 0,
      .who=0,
  };
  void iterate();

  /**
   * @brief config the characteristic for BLE
   * @param[in] server
   * @param[out] ble the LaneBLE to be written, expected to be initialized
   * @warning `ctrl_cb` and `config_cb` are static variables, so they are initialized only once.
   * It would be a problem if you have multiple lanes. However, nobody would do that.
   */
  static void _initBLE(NimBLEServer &server, LaneBLE &ble);

  void stop() const;

public:
  explicit Lane(strip_ptr_t strip) : strip(std::move(strip)){};
  [[nodiscard]] meter lengthPerLED() const;
  [[nodiscard]] auto getLaneLEDsNum() const {
    return this->cfg.line_LEDs_num;
  }

  /**
   * @brief Initialize the BLE service.
   * @param server the BLE server
   * @warning
   */
  void initBLE(NimBLEServer &server) {
    if (this->ble.service != nullptr) {
      ESP_LOGE("LANE", "BLE has already been initialized");
      return;
    }
    _initBLE(server, ble);
  }

  /**
   * @brief Loop the strip.
   * @warning This function will never return and you should call this in creatTask/Thread
   *          or something equivalent in RTOS.
   * @return No return
   */
  [[noreturn]] void loop();


  void oled_init(int mode) {

    Wire.begin();
    u8g2.begin();
    u8g2.setFont(u8g2_font_10x20_t_greek);
    this->mode_update(mode);
  }


void scan_key(uint8_t key);

  uint32_t tick_buf = 0;
  uint16_t length_list[12] = {50,100,200,400,800,1500,50,100,200,400,800,1500};
  uint16_t   time_list[24] = {24,27,51,65,123,143,261,306,542,632,1040,1215,
                          27,31,62,73,135,159,264,346,582,722,1115,1425};
  void mode_update(int mode){
    if(mode<0 or mode > 23){
      ESP_LOGI("oled",  "customize mode");
      this->setMode(mode);

      u8g2.clearBuffer();
      u8g2.drawStr(2,20,"CUSTOM:");
      u8g2.drawStr(80,20 ,std::to_string(cfg.finish_length.count()).c_str());
      u8g2.drawStr(2,50,"  Time:");
      u8g2.drawStr(80,50 ,std::to_string(cfg.finish_time).c_str());
      u8g2.sendBuffer();
    }else {
      ESP_LOGI("oled",  "mode_update");
      this->setMode(mode);
      this->setPaceNum(1);
      const uint16_t new_length = length_list[mode/2];
      const uint16_t new_time = time_list[mode];
      setFinishLength(meter(new_length));
      setFinishTime(new_time);
      setStatus(LaneStatus::STOP);
      u8g2.clearBuffer();
      if(mode < 12) {
        u8g2.drawStr(2,20,"  Male:");
      }else {
        u8g2.drawStr(2,20,"Female:");
      }
      u8g2.drawStr(80,20 ,std::to_string(new_length).c_str());
      u8g2.drawStr(2,50,"  Time:");
      u8g2.drawStr(80,50 ,std::to_string(new_time).c_str());
      u8g2.sendBuffer();
    }
      this->setActMode(ActMode::TIME_RELY);
      pref.begin(common::lanely::PREF_RECORD_NAME, false);
      pref.putUChar("mode", pace.pace_mode);
      Serial.printf("new mode saved:%d\r\n",pace.pace_mode);
      pref.end();
  }
void single_click_f(uint8_t key){
  key_char = key_char_zero;
  auto mode = pace.pace_mode;
  switch(key){
    case key_down:
      mode++;
      if(pace.pace_mode == 11){
        mode = 0;
      }else if(pace.pace_mode == 23){
        mode = 12;
      }else if(pace.pace_mode > 23) {
        mode = 0;
      }
      mode_update(mode);
    break;
    case key_up:
      mode--;
      if(pace.pace_mode == 0){
        mode = 11;
      }else if(pace.pace_mode == 12){
        mode = 23;
      }else if(pace.pace_mode > 23) {
        mode = 0;
      }
      mode_update(mode);
    break;
  }
}

void double_click_f(uint8_t key){
  key_char = key_char_zero;
}

void long_click_f(uint8_t key){
  key_char =key_char_zero;
  switch(key){
    case key_down:
      if(this->params.status == LaneStatus::STOP){
        this->setStatus(LaneStatus::FORWARD);
      }else {
        this->setStatus(LaneStatus::STOP);
      }
    break;
    case key_up:
      if(pace.pace_mode <= 11){
        auto mode = pace.pace_mode + 12;
        mode_update(mode);
      }else{
        auto mode = pace.pace_mode-12;
        mode_update(mode);
      }
    break;
  }
}

  /**
   * @brief sets the maximum number of LEDs that can be used. i.e. Circle Length.
   * @warning This function will NOT set the corresponding bluetooth characteristic value.
   * @param new_max_LEDs the new maximum number of LEDs
   */
  void setMaxLEDs(uint32_t new_max_LEDs);

  /**
   * @brief set the status of the strip.
   * @warning This function WILL set the corresponding bluetooth characteristic value and notify.
   * @param st
   */
  void notifyState(LaneState st);

  esp_err_t begin();

  void setConfig(const LaneConfig &newCfg) {
    this->cfg = newCfg;
  };

  void setFinishLength(const meter newlength){
    if(newlength < meter(50) or newlength>meter(3000)){
      Serial.println("Invalid length");
      return;
    }else{
      this->cfg.finish_length = newlength;
    }
  }

  void setFinishTime(const int time_sec){
    if(time_sec < 0 or time_sec >65535){
      Serial.println("Invalid time");
      return;
    }else{
      this->cfg.finish_time = time_sec;
    }
  }

  void setSpeed(const float speed) {
    this->params.speed = speed;
  };

  void setActMode(const ActMode newActMode) {
    this->params.act_mode = newActMode;
  }

  void showTestSpeed(const float Nspeed) {
    u8g2.clearBuffer();
    u8g2.drawStr(2,30,"TESTING");
    //u8g2.drawStr(80,20 ,std::to_string(cfg.finish_length.count()).c_str());
    u8g2.drawStr(2,50,"SPEED :");
    u8g2.drawStr(80,50 ,std::to_string(params.speed).c_str());
    u8g2.sendBuffer();
  }

  void setMode(uint8_t newMode){
    this->pace.pace_mode = newMode;
  }

  void setPaceNum(uint8_t newnum){
    pace.pace_num = newnum;
  }

  void setPaceTime0(float newTime){
    this->pace.pace_time[0] = newTime;
  }

  void set_pause_tick(int tick){
  if(tick<0){
    return;
  }else{
    this->params.pause_tick = tick;
  }
}

  void setColor(const uint32_t color) {
    this->cfg.color = color;
  };

  void setStatus(const LaneStatus status) {
    this->params.status = static_cast<LaneStatus>(status);
    this->pace.start_time = 0;
    this->params.speed = 0;
    this->state = LaneState::zero();
    if(this->params.status == LaneStatus::STOP){
      this->pace.start_time = 0;
      this->params.speed = 0;
      this->state = LaneState::zero();
    }
  };

  void setStartTime() {
    this->pace.start_time= millis();
  }
LanePace pace   = {
      .pace_num = 10,
      .pace_time = {33,0,0,0,0},
      .else_deal  = 1,
      .platform_surface_time = 5,
      .platform_surface_range = 10,
      .acceleration = 1,
      .turn_time = 1 ,
      .start_time = 0,
      .pace_mode  =0,
  };

void setPace(const LanePace &newPace){
  this->pace = newPace;
}
  void setStatus(::LaneStatus status) {
    this->params.status = static_cast<LaneStatus>(status);
    this->pace.start_time = 0;
      this->params.speed = 0;
      this->state = LaneState::zero();
    if(this->params.status == LaneStatus::STOP){
      this->pace.start_time = 0;
      this->params.speed = 0;
      this->state = LaneState::zero();
    }
  }

  float make_contain(float in, float standard, float offset);

  void update_speed(const LaneState &last_state, const LaneConfig &cfg, const LaneParams &input ,const LanePace &pace);
  inline void resetDecodeBuffer() {
    decode_buffer.fill(0);
  }
  [[nodiscard]] float LEDsPerMeter() const;
};

//*********************************** Callbacks ****************************************/

};

#endif // HELLO_WORLD_STRIP_H
