//
// Created by Kurosu Chan on 2023/10/27.
//

#include "utils.h"

namespace utils {

size_t sprintHex(char *out, size_t outSize, const uint8_t *bytes, size_t size) {
  size_t i = 0;
  // 2 hex chars
  if (outSize < (size * 2)) {
    return 0;
  }
  while (i < (size * 2)) {
    // consider endianness
    const uint8_t byte   = bytes[i / 2];
    const uint8_t nibble = (i % 2 == 0) ? (byte >> 4) : (byte & 0x0F);
    out[i++]             = (nibble < 10) ? ('0' + nibble) : ('a' + nibble - 10);
  }
  return i;
};

std::string toHex(const uint8_t *bytes, size_t size) {
  auto sizeNeeded = size * 2;
  auto res        = std::string(sizeNeeded, '\0');
  auto _          = sprintHex(res.data(), sizeNeeded, bytes, size);
  static_cast<void>(_);
  return res;
}
}
