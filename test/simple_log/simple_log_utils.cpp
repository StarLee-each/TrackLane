//
// Created by Kurosu Chan on 2023/10/17.
//
#include "simple_log_utils.h"


namespace utils{
size_t sprintHex(char *out, size_t outSize, const uint8_t *bytes, size_t size) {
  size_t i = 0;
  // 2 hex chars + 1 null terminator
  if (outSize < (size * 2)) {
    return 0;
  }
  while (i < (size * 2)) {
    // consider endianness
    uint8_t byte   = bytes[i / 2];
    uint8_t nibble = (i % 2 == 0) ? (byte >> 4) : (byte & 0x0F);
    out[i++]       = (nibble < 10) ? static_cast<char>(('0' + nibble)) : static_cast<char>(('a' + nibble - 10));
  }
  out[i] = '\0';
  return i;
};

std::string toHex(const uint8_t *bytes, size_t size) {
  auto sizeNeeded = size * 2;
  auto res        = std::string(sizeNeeded, 0x00);
  auto len        = sprintHex(const_cast<char *>(res.data()), sizeNeeded, bytes, size);
  return res;
};
}
