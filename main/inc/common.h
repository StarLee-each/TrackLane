//
// Created by Kurosu Chan on 2023/11/6.
//

#ifndef TRACK_SHORT_COMMON_H
#define TRACK_SHORT_COMMON_H
#include <driver/gpio.h>
#include "utils.h"
namespace common {
constexpr auto BLE_NAME = "lane";

constexpr auto BLE_SERVICE_UUID         = "15ce51cd-7f34-4a66-9187-37d30d3a1464";
constexpr auto BLE_CHAR_HR_SERVICE_UUID = "180d";

constexpr auto BLE_CHAR_CONTROL_UUID    = "24207642-0d98-40cd-84bb-910008579114";
constexpr auto BLE_CHAR_CONFIG_UUID     = "e89cf8f0-7b7e-4a2e-85f4-85c814ab5cab";
constexpr auto BLE_CHAR_HEARTBEAT_UUID  = "048b8928-d0a5-43e2-ada9-b925ec62ba27";
constexpr auto BLE_CHAR_PACE_UUID       = "54f5e01d-eb14-420f-8ed1-7b453ba526cb";

constexpr auto BLE_CHAR_WHITE_LIST_UUID = "12a481f0-9384-413d-b002-f8660566d3b0";
constexpr auto BLE_CHAR_DEVICE_UUID     = "a2f05114-fdb6-4549-ae2a-845b4be1ac48";

/**
 * @brief some common *constant* definitions for `lane`
 * @note it's called `lanely` since `lane` the namespace has been taken in global namespace
 */
namespace lanely {
  using centimeter                       = utils::length<float, std::centi>;
  using meter                            = utils::length<float, std::ratio<1>>;
  constexpr auto PREF_RECORD_NAME        = "rec";
  constexpr auto PREF_LINE_LENGTH_NAME   = "ll";
  constexpr auto PREF_ACTIVE_LENGTH_NAME = "al";
  constexpr auto PREF_LINE_LEDs_NUM_NAME = "ln";
  constexpr auto PREF_TOTAL_LENGTH_NAME  = "to"; // float
  constexpr auto PREF_COLOR_NAME         = "co"; // uint32_t

  constexpr auto DEFAULT_ACTIVE_LENGTH  = meter(0.6);  // the line would be active for this length
  constexpr auto DEFAULT_LINE_LENGTH    = meter(50);   // line... it would wrap around
  constexpr auto DEFAULT_TARGET_LENGTH  = meter(1000); // like shift
  constexpr auto DEFAULT_LINE_LEDs_NUM  = static_cast<uint32_t>(DEFAULT_LINE_LENGTH.count() * (100 / 3.3));
  constexpr auto DEFAULT_FPS  = 10;
  constexpr auto DEFAULT_FINISH_TIME= 900;
  constexpr size_t DECODE_BUFFER_SIZE   = 2048;
  constexpr auto BLUE_TRANSMIT_INTERVAL = std::chrono::milliseconds(1000);
  constexpr auto HALT_INTERVAL          = std::chrono::milliseconds(500);
  constexpr neoPixelType PIXEL_TYPE     = NEO_RGB + NEO_KHZ800;
}

namespace pin {
  constexpr auto LED      = GPIO_NUM_23;
  constexpr auto SCK      = GPIO_NUM_16;
  constexpr auto NSS      = GPIO_NUM_4;
  constexpr auto MOSI     = GPIO_NUM_17;
  constexpr auto MISO     = GPIO_NUM_5;
  constexpr auto LoRa_RST = GPIO_NUM_18;
  constexpr auto BUSY     = GPIO_NUM_19;
  constexpr auto DIO1     = GPIO_NUM_21;
  constexpr auto DIO2     = GPIO_NUM_33;
  constexpr auto DIO3     = GPIO_NUM_26;
}
};

#endif // TRACK_SHORT_COMMON_H
