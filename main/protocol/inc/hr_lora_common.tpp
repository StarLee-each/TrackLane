//
// Created by Kurosu Chan on 2023/11/2.
//

#ifndef BLE_LORA_ADAPTER_HR_LORA_COMMON_H
#define BLE_LORA_ADAPTER_HR_LORA_COMMON_H

/**
 * @brief some common *constant* definitions for HRLoRA
 */

#include <string>
#include <etl/optional.h>

namespace HrLoRa {
constexpr auto BLE_ADDR_SIZE  = 6;
constexpr auto broadcast_addr = std::array<uint8_t, BLE_ADDR_SIZE>{0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
using name_map_key_t          = uint8_t;
using addr_t                  = std::array<uint8_t, BLE_ADDR_SIZE>;

#if __cplusplus >= 202002L
/**
 * @brief module struct is a struct that has a special struct `t`
 *       and all the other members and functions are static, inspired by OCaml
 * @tparam T
 */
template <typename T>
concept module_struct = requires {
  // has a special member type `t`
  typename T::t;
  // the special member has a member type `module` that refers to the module struct itself
  typename T::t::module;
  requires std::same_as<typename T::t::module, T>;
  // strictly, it should not have any non-static data member (i.e. empty)
  // https://en.cppreference.com/w/cpp/types/is_empty
  requires std::is_empty_v<T>;
};

template <typename T>
concept _marshallable = requires(const T::t &t, uint8_t *buffer, size_t size) {
  { T::marshal(t, buffer, size) } -> std::convertible_to<size_t>;
};

/**
 * @brief a concept to check if a module struct is marshallable
 * @tparam T a module struct that has a special member type `t` and a static function `marshal`
 */
template <typename T>
concept marshallable = module_struct<T> && _marshallable<T>;

template <typename T>
concept _unmarshallable = requires(const uint8_t *buffer, size_t size) {
  { T::unmarshal(buffer, size) } -> std::convertible_to<etl::optional<typename T::t>>;
};

/**
 * @brief a concept to check if a module struct is unmarshallable
 * @tparam T a module struct that has a actual member type `t` and a static function `unmarshal`
 */
template <typename T>
concept unmarshallable = module_struct<T> && _unmarshallable<T>;
#endif
}

#endif // BLE_LORA_ADAPTER_HR_LORA_COMMON_H
