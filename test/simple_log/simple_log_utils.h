//
// Created by Kurosu Chan on 2023/10/17.
//

#ifndef NANOPB_TEST_SIMPLE_LOG_UTILS_H
#define NANOPB_TEST_SIMPLE_LOG_UTILS_H
#include <iostream>

namespace utils {
size_t sprintHex(char *out, size_t outSize, const uint8_t *bytes, size_t size);
std::string toHex(const uint8_t *bytes, size_t size);
}

#endif // NANOPB_TEST_SIMPLE_LOG_UTILS_H
