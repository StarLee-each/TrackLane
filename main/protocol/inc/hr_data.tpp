//
// Created by Kurosu Chan on 2023/11/2.
//

#ifndef BLE_LORA_ADAPTER_HR_DATA_H
#define BLE_LORA_ADAPTER_HR_DATA_H

#include <string>
#include <etl/optional.h>

namespace HrLoRa {
struct hr_data {
  static constexpr uint8_t magic = 0x63;
  struct t {
    using module = hr_data;
    uint8_t key  = 0;
    uint8_t hr   = 0;
  };
#if __cpp_consteval >= 202002L
  consteval
#else
  constexpr
#endif
  static size_t size_needed() {
    // key + hr + magic
    return sizeof(magic) + sizeof(t::key) + sizeof(t::hr);
  }
  static size_t marshal(const t &data, uint8_t *buffer, size_t buffer_size) {
    if (buffer_size < size_needed()) {
      return 0;
    }
    buffer[0] = magic;
    buffer[1] = data.key;
    buffer[2] = data.hr;
    return size_needed();
  }
  static etl::optional<t> unmarshal(const uint8_t *buffer, size_t size) {
    if (size < size_needed()) {
      return etl::nullopt;
    }

    t data;
    if (buffer[0] != magic) {
      return etl::nullopt;
    }

    data.key = buffer[1];
    data.hr  = buffer[2];

    return data;
  }
};
}

#endif // BLE_LORA_ADAPTER_HR_DATA_H
