//
// Created by Kurosu Chan on 2023/11/2.
//

#ifndef BLE_LORA_ADAPTER_SET_NAME_MAP_KEY_H
#define BLE_LORA_ADAPTER_SET_NAME_MAP_KEY_H

#include <string>
#include <etl/optional.h>
#include "hr_lora_common.tpp"

namespace HrLoRa {
struct set_name_map_key {
  static constexpr uint8_t magic = 0x79;
  struct t {
    using module = set_name_map_key;
    addr_t addr{};
    name_map_key_t key = 0;
  };

#if __cpp_consteval >= 202002L
  consteval
#else
  constexpr
#endif
  static size_t size_needed() {
    // addr + key + magic
    return BLE_ADDR_SIZE + sizeof(t::key) + sizeof(magic);
  }
  static size_t marshal(const t &data, uint8_t *buffer, size_t buffer_size) {
    if (buffer_size < size_needed()) {
      return 0;
    }
    buffer[0] = magic;
    for (int i = 0; i < BLE_ADDR_SIZE; ++i) {
      buffer[i + 1] = data.addr[i];
    }
    buffer[BLE_ADDR_SIZE + 1] = data.key;
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

    for (int i = 0; i < BLE_ADDR_SIZE; ++i) {
      data.addr[i] = buffer[i + 1];
    }
    data.key = buffer[BLE_ADDR_SIZE + 1];

    return data;
  }
};
}

#endif // BLE_LORA_ADAPTER_SET_NAME_MAP_KEY_H
