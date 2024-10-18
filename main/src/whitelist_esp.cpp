//
// Created by Kurosu Chan on 2023/10/16.
//

#include "whitelist.h"

namespace white_list {
bool is_device_in_whitelist(const item_t &item, BLEAdvertisedDevice &device) {
  return std::visit(IsDeviceVisitor{device}, item);
}
}
