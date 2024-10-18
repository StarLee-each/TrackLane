//
// Created by Kurosu Chan on 2023/11/2.
//

#ifndef BLE_LORA_ADAPTER_QUERY_DEVICE_BY_MAC_H
#define BLE_LORA_ADAPTER_QUERY_DEVICE_BY_MAC_H

#include <string>
#include <etl/optional.h>
#include "hr_lora_common.tpp"

namespace HrLoRa {
struct query_device_by_mac {
  struct t {
    using module = query_device_by_mac;
    addr_t addr{};
  };
#if __cpp_consteval >= 202002L
  consteval
#else
  constexpr
#endif
  static size_t size_needed() {
    return BLE_ADDR_SIZE + 1;
  }
  static constexpr addr_t broadcast_addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  static constexpr uint8_t magic         = 0x37;
  static size_t marshal(const t &data, uint8_t *buffer, size_t size) {
    if (size < size_needed()) {
      return 0;
    }
    buffer[0] = magic;
    for (int i = 0; i < BLE_ADDR_SIZE; ++i) {
      buffer[i + 1] = data.addr[i];
    }
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

    return data;
  }
};
}

#endif // BLE_LORA_ADAPTER_QUERY_DEVICE_BY_MAC_H
