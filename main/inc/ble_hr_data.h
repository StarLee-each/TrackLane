//
// Created by Kurosu Chan on 2023/11/7.
//

#ifndef TRACK_SHORT_BLE_HR_DATA_H
#define TRACK_SHORT_BLE_HR_DATA_H
#include <array>
#include <string>
#include <etl/optional.h>

namespace ble {
static const auto BLE_ADDR_SIZE = 6;
using addr_t                    = std::array<uint8_t, BLE_ADDR_SIZE>;
// TODO: use Bluetooth LE address instead of name since name is not unique
// open a characteristic to get the name of the device
struct hr_data {
  struct t {
    using module = hr_data;
    std::string name{};
    uint8_t hr = 0;
  };
  static size_t size_needed(const t &data) {
    return BLE_ADDR_SIZE + 1 + data.name.size();
  }
  static size_t marshal(const t &data, uint8_t *buffer, size_t size) {
    if (size < size_needed(data)) {
      return 0;
    }
    size_t offset    = 0;
    buffer[offset++] = static_cast<uint8_t>(data.name.size());
    for (char i : data.name) {
      buffer[offset++] = i;
    }
    buffer[offset++] = data.hr;
    return offset;
  }

  static etl::optional<t> unmarshal(const uint8_t *buffer, size_t buffer_size) {
    if (buffer_size < BLE_ADDR_SIZE) {
      return etl::nullopt;
    }
    t data;
    size_t sz = buffer[0];
    data.name = std::string(reinterpret_cast<const char *>(buffer + BLE_ADDR_SIZE + 1), sz);
    data.hr   = buffer[BLE_ADDR_SIZE + 1 + sz];
    return data;
  }
};
}

#endif // TRACK_SHORT_BLE_HR_DATA_H
