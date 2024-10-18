//
// Created by Kurosu Chan on 2023/7/31.
//

#include <cstdio>
#include <pb_common.h>
#include <pb_decode.h>
#include "Lane.h"
#include "lane.pb.h"
#include "utils.h"
#include "ble.pb.h"

//****************************** Callback ************************************/

namespace lane {
void Lane::ControlCharCallback::onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo) {
  auto TAG                  = "control";
  auto data                 = characteristic->getValue();
  ::LaneControl control_msg = LaneControl_init_zero;
  auto ostream              = pb_istream_from_buffer(data.data(), data.size());
  auto ok                   = pb_decode(&ostream, LaneControl_fields, &control_msg);
  if (!ok) {
    ESP_LOGE(TAG, "Failed to decode the control message");
    return;
  }
  switch (control_msg.which_msg) {
    case LaneControl_set_speed_tag:
      ESP_LOGI(TAG, "Set speed to %f", control_msg.msg.set_speed.speed);
      lane.setSpeed(control_msg.msg.set_speed.speed);
      break;
    case LaneControl_set_status_tag:
      ESP_LOGI(TAG, "Set status to %d", control_msg.msg.set_status.status);
      lane.setStatus(control_msg.msg.set_status.status);
      break;
    default:
      ESP_LOGE(TAG, "Unknown message type");
      break;
  }
}

void Lane::ConfigCharCallback::onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo) {
  using namespace common::lanely;
  const auto TAG          = "config::write";
  auto data               = characteristic->getValue();
  ::LaneConfig config_msg = LaneConfig_init_zero;
  auto ostream            = pb_istream_from_buffer(data.data(), data.size());
  auto ok                 = pb_decode(&ostream, LaneConfig_fields, &config_msg);

  if (!ok) {
    ESP_LOGE("LANE", "Failed to decode the config message");
    return;
  }
  lane.pref.begin(PREF_RECORD_NAME, false);
  switch (config_msg.which_msg) {
    case LaneConfig_color_cfg_tag:
      lane.pref.putULong(PREF_COLOR_NAME, config_msg.msg.color_cfg.rgb);
      ESP_LOGI(TAG, "Set color to 0x%06lx", config_msg.msg.color_cfg.rgb);
      lane.setColor(config_msg.msg.color_cfg.rgb);
      break;
    case LaneConfig_length_cfg_tag: {
      if (lane.state.status != LaneStatus::STOP) {
        ESP_LOGE(TAG, "Can't change the length while the lane is running");
        return;
      }
      ESP_LOGI(TAG, "line length=%.2f; active length=%.2f; total length=%.2f; line LEDs=%ld; finish time=%d",
               config_msg.msg.length_cfg.line_length_m,
               config_msg.msg.length_cfg.active_length_m,
               config_msg.msg.length_cfg.total_length_m,
               config_msg.msg.length_cfg.line_leds_num,
               config_msg.msg.length_cfg.finish_time_m);
      lane.pref.putFloat(PREF_LINE_LENGTH_NAME, config_msg.msg.length_cfg.line_length_m);
      lane.pref.putFloat(PREF_ACTIVE_LENGTH_NAME, config_msg.msg.length_cfg.active_length_m);
      lane.pref.putFloat(PREF_TOTAL_LENGTH_NAME, config_msg.msg.length_cfg.total_length_m);
      lane.pref.putULong(PREF_LINE_LEDs_NUM_NAME, config_msg.msg.length_cfg.line_leds_num);
      lane.cfg.line_length   = meter(config_msg.msg.length_cfg.line_length_m);
      lane.cfg.active_length = meter(config_msg.msg.length_cfg.active_length_m);
      lane.cfg.finish_length = meter(config_msg.msg.length_cfg.total_length_m);
      lane.cfg.line_LEDs_num = config_msg.msg.length_cfg.line_leds_num;
      lane.cfg.finish_time = config_msg.msg.length_cfg.finish_time_m;
      lane.setMaxLEDs(config_msg.msg.length_cfg.line_leds_num);
      break;
    }
  }
  lane.pref.end();
}
void Lane::ConfigCharCallback::onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) {
  const auto TAG        = "config::read";
  constexpr size_t size = LaneConfigRO_size + 16;
  uint8_t data[size];
  ::LaneConfigRO config_msg = LaneConfigRO_init_zero;
  auto ostream              = pb_ostream_from_buffer(data, size);
  // https://stackoverflow.com/questions/56661663/nanopb-encode-always-size-0-but-no-encode-failure
  config_msg.has_color_cfg              = true;
  config_msg.has_length_cfg             = true;
  config_msg.length_cfg.line_length_m   = lane.cfg.line_length.count();
  config_msg.length_cfg.active_length_m = lane.cfg.active_length.count();
  config_msg.length_cfg.total_length_m  = lane.cfg.finish_length.count();
  config_msg.length_cfg.line_leds_num   = lane.cfg.line_LEDs_num;
  config_msg.color_cfg.rgb              = lane.cfg.color;
  ESP_LOGI(TAG, "line length=%.2f; active length=%.2f; total length=%.2f; line LEDs=%ld; Color=0x%06lx",
           config_msg.length_cfg.line_length_m, config_msg.length_cfg.active_length_m,
           config_msg.length_cfg.total_length_m, config_msg.length_cfg.line_leds_num,
           config_msg.color_cfg.rgb);
  if (const auto ok = pb_encode(&ostream, LaneConfigRO_fields, &config_msg); !ok) {
    ESP_LOGE(TAG, "encode: %s", PB_GET_ERROR(&ostream));
    return;
  }

  ESP_LOGD(TAG, "encoded(%d): %s", ostream.bytes_written, utils::toHex(data, ostream.bytes_written).c_str());

  pCharacteristic->setValue(data, ostream.bytes_written);
}

void Lane::PaceCharCallback::onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo){
  using namespace common::lanely;
  const auto TAG          = "pace::write";
  auto data               = characteristic->getValue();
  ::PbLanePace pace_msg = PbLanePace_init_zero;
  auto istream            = pb_istream_from_buffer(data.data(), data.size());
  auto ok                 = pb_decode( &istream, PbLanePace_fields, &pace_msg);

  if (!ok) {
    ESP_LOGE("LANE", "Failed to decode the pace message");
    return;
  }
  ESP_LOGI(TAG, "turn time=%.2f; acceleration=%.2f;",pace_msg.turn_time,pace_msg.accel);
  ESP_LOGI(TAG, "pace each init:");
  for(auto i=0;i<pace_msg.pace_time_pct_count;i++){
    printf("stage %d : %.2f %\r\n",i,pace_msg.pace_time_pct[i]);
  }
  memcpy(lane.pace.pace_time_pct,pace_msg.pace_time_pct,pace_msg.pace_time_pct_count);
  lane.pace.pace_num = pace_msg.pace_time_pct_count;
  lane.pace.turn_time= pace_msg.turn_time;
  lane.pace.acceleration = pace_msg.accel;
}
};
