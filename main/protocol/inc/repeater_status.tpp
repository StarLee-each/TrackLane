//
// Created by Kurosu Chan on 2023/11/7.
//

#ifndef BLE_LORA_ADAPTER_REPEATER_STATUS_H
#define BLE_LORA_ADAPTER_REPEATER_STATUS_H

namespace HrLoRa {
struct hr_device {
  struct t {
    using module = hr_device;
    addr_t addr{};
    // zero terminated string
    std::string name{};
  };
  static size_t size_needed(const t &data) {
    return BLE_ADDR_SIZE + data.name.size() + 1;
  }
  static size_t marshal(const t &data, uint8_t *buffer, size_t size) {
    if (size < size_needed(data)) {
      return 0;
    }
    size_t offset = 0;
    for (int i = 0; i < BLE_ADDR_SIZE; ++i) {
      buffer[offset++] = data.addr[i];
    }
    for (char i : data.name) {
      buffer[offset++] = i;
    }
    buffer[offset++] = 0;
    return offset;
  }
  static etl::optional<t> unmarshal(const uint8_t *buffer, size_t buffer_size) {
    if (buffer_size < BLE_ADDR_SIZE) {
      return etl::nullopt;
    }

    t data;
    for (int i = 0; i < BLE_ADDR_SIZE; ++i) {
      data.addr[i] = buffer[i];
    }
    size_t offset = BLE_ADDR_SIZE;
    data.name     = std::string(reinterpret_cast<const char *>(buffer + offset));
    return data;
  }
};

struct repeater_status {
  static constexpr uint8_t magic = 0x47;
  struct t {
    using module = repeater_status;
    addr_t repeater_addr{};
    name_map_key_t key                 = 0;
    etl::optional<hr_device::t> device = etl::nullopt;
  };
  static size_t size_needed(const t &data) {
    return sizeof(magic) +
           BLE_ADDR_SIZE +
           sizeof(t::key) +
           (data.device ? hr_device::size_needed(*data.device) : 0);
  }
  static size_t marshal(const t &data, uint8_t *buffer, size_t size) {
    if (size < size_needed(data)) {
      return 0;
    }
    size_t offset    = 0;
    buffer[offset++] = magic;
    for (int i = 0; i < BLE_ADDR_SIZE; ++i) {
      buffer[offset++] = data.repeater_addr[i];
    }
    buffer[offset++] = data.key;
    // last bit indicates whether there is a device
    uint8_t flag = 0x00;
    if (data.device) {
      flag |= 0x01;
    }
    buffer[offset++] = flag;
    if (data.device) {
      offset += hr_device::marshal(*data.device, buffer + offset, size - offset);
    }
    return offset;
  }
  static etl::optional<t> unmarshal(const uint8_t *buffer, size_t size) {
    if (size < BLE_ADDR_SIZE + sizeof(name_map_key_t)) {
      return etl::nullopt;
    }

    t data;
    if (buffer[0] != magic) {
      return etl::nullopt;
    }
    size_t offset = 1;
    for (int i = 0; i < BLE_ADDR_SIZE; ++i) {
      data.repeater_addr[i] = buffer[offset++];
    }
    data.key     = buffer[offset++];
    uint8_t flag = buffer[offset++];
    if (flag & 0x01) {
      data.device = hr_device::unmarshal(buffer + offset, size - offset);
    } else {
      data.device = etl::nullopt;
    }
    return data;
  }
};
}

#endif // BLE_LORA_ADAPTER_REPEATER_STATUS_H
