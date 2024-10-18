//
// Created by Kurosu Chan on 2023/11/2.
//

#ifndef BLE_LORA_ADAPTER_HR_LORA_H
#define BLE_LORA_ADAPTER_HR_LORA_H

/**
 * @note user should only include this file
 *       don't include any other files in this directory
 */

#include "hr_lora_common.tpp"
#include "hr_data.tpp"
#include "query_device_by_mac.tpp"
#include "set_name_map_key.tpp"
#include "named_hr_data.tpp"
#include "repeater_status.tpp"

namespace HrLoRa::hr_lora_msg {
using t = std::variant<
    named_hr_data::t,
    hr_data::t,
    query_device_by_mac::t,
    repeater_status::t,
    set_name_map_key::t>;

// https://en.cppreference.com/w/cpp/utility/variant/visit
// helper constant for the visitor #3
template <class>
inline constexpr bool always_false_v = false;
// helper type for the visitor #4
template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};
// explicit deduction guide (not needed as of C++20)
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

size_t marshal(t &data, uint8_t *buffer, size_t size) {
  return std::visit(overloaded{
                        [buffer, size](named_hr_data::t &data) {
                          return named_hr_data::marshal(data, buffer, size);
                        },
                        [buffer, size](hr_data::t &data) {
                          return hr_data::marshal(data, buffer, size);
                        },
                        [buffer, size](query_device_by_mac::t &data) {
                          return query_device_by_mac::marshal(data, buffer, size);
                        },
                        [buffer, size](repeater_status::t &data) {
                          return repeater_status::marshal(data, buffer, size);
                        },
                        [buffer, size](set_name_map_key::t &data) {
                          return set_name_map_key::marshal(data, buffer, size);
                        },
                    },
                    data);
}

// https://stackoverflow.com/questions/55612759/alternative-to-using-namespace-as-template-parameter
// https://caml.inria.fr/pub/docs/oreilly-book/html/book-ora132.html
// However, namespace could be treated as a struct
#if __cplusplus >= 202002L
template <unmarshallable T>
#else
template <typename T>
#endif
etl::optional<t> unmarshal_helper(const uint8_t *buffer, size_t size) {
  auto res = T::unmarshal(buffer, size);
  if (res) {
    return t{res.value()};
  } else {
    return etl::nullopt;
  }
}

etl::optional<t> unmarshal(const uint8_t *buffer, size_t size) {
  if (size < 1) {
    return etl::nullopt;
  }
  auto magic = buffer[0];
  switch (magic) {
    case named_hr_data::magic: {
      return unmarshal_helper<named_hr_data>(buffer, size);
    }
    case hr_data::magic: {
      return unmarshal_helper<hr_data>(buffer, size);
    }
    case query_device_by_mac::magic: {
      return unmarshal_helper<query_device_by_mac>(buffer, size);
    }
    case repeater_status::magic: {
      return unmarshal_helper<repeater_status>(buffer, size);
    }
    case set_name_map_key::magic: {
      return unmarshal_helper<set_name_map_key>(buffer, size);
    }
    default:
      return etl::nullopt;
  }
}
}

#endif // BLE_LORA_ADAPTER_HR_LORA_H
