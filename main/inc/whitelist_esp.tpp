//
// Created by Kurosu Chan on 2023/10/16.
//

#ifndef TRACK_SHORT_WHITELIST_ESP_H
#define TRACK_SHORT_WHITELIST_ESP_H

#include <NimBLEDevice.h>

namespace white_list {
// https://stackoverflow.com/questions/75278137/correct-use-of-stdvariant-and-stdvisit-when-functor-requires-multiple-argume
    struct IsDeviceVisitor {
      BLEAdvertisedDevice &device;
      bool operator()(const white_list::Name &name) const {
        const auto device_name = device.getName();
        if (device_name.empty()) {
          return false;
        }
        auto re = std::regex(name.name);
        return std::regex_match(device_name, re);
      }

      bool operator()(const white_list::Addr &addr) const {
        // a pointer to the uint8_t[6] array of the address
        const auto device_addr = device.getAddress().getNative();
        return std::memcmp(addr.addr.data(), device_addr, BLE_MAC_ADDR_SIZE) == 0;
      }
    };
    bool is_device_in_whitelist(const item_t &item, BLEAdvertisedDevice &device);
}

#endif // TRACK_SHORT_WHITELIST_ESP_H
