//
// Created by Kurosu Chan on 2023/11/7.
//

#ifndef TRACK_SHORT_HR_DATA_WITH_NAME_H
#define TRACK_SHORT_HR_DATA_WITH_NAME_H

namespace HrLoRa {
struct named_hr_data {
  static constexpr uint8_t magic = 0x60;
  struct t {
    using module = named_hr_data;
    uint8_t key  = 0;
    uint8_t hr   = 0;
    addr_t addr{};
  };

#if __cpp_consteval >= 202002L
  consteval
#else
  constexpr
#endif
  static size_t size_needed() {
    // key + hr + magic
    return sizeof(magic) + sizeof(t::key) + sizeof(t::hr) + BLE_ADDR_SIZE;
  }
  static size_t marshal(const t &data, uint8_t *buffer, size_t buffer_size) {
    if (buffer_size < size_needed()) {
      return 0;
    }
    size_t offset    = 0;
    buffer[offset++] = magic;
    buffer[offset++] = data.key;
    buffer[offset++] = data.hr;
    std::copy(data.addr.begin(), data.addr.end(), buffer + offset);
    offset += BLE_ADDR_SIZE;
    return offset;
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
    std::copy(buffer + 3, buffer + 3 + BLE_ADDR_SIZE, data.addr.begin());

    return data;
  }
};
}

#endif // TRACK_SHORT_HR_DATA_WITH_NAME_H
