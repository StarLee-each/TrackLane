//
// Created by Kurosu Chan on 2022/8/4.
//
#ifndef HELLO_WORLD_UTILS_H
#define HELLO_WORLD_UTILS_H

#include <chrono>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <vector>
#include <pb_encode.h>
#include <map>
#include <esp_timer.h>

#if __cplusplus >= 202002L
//#include <concepts>
#endif

/**
 * @brief a function retrieve value by the number nearing the key. always move a unit up.
 *
 * @tparam T the type of value of map
 * @param keys a std::vector of int which should be sorted
 * @param m  a map whose key is int
 * @param val a value you want
 * @return T
 */
template <typename T>
T retrieveByVal(std::vector<int> const &keys, std::map<int, T> const &m, int val) {
  size_t idx = 0;
  // since keys is sorted we can get result easily
  for (auto key : keys) {
    if (val < key) {
      break;
    }
    idx += 1;
  }
  if (idx >= keys.size()) {
    idx = keys.size() - 1;
  }
  return m.at(keys[idx]);
}

template <class T>
class ValueRetriever {
private:
  std::map<int, T> m;
  std::vector<int> keys;
  size_t max_key{};

public:
  [[nodiscard]] decltype(max_key) getMaxKey() const {
    return max_key;
  }

  const std::map<int, T> &getMap() const {
    return m;
  }

  [[nodiscard]] const std::vector<int> &getKeys() const {
    return keys;
  }

  /**
   * @brief Construct a new Value Retriever object
   *
   * @param m the map whose keys are int. The map should be immutable or call [updateKeys] after changing,
   *        otherwise things will be weired.
   */
  explicit ValueRetriever(std::map<int, T> m) : m(m) {
    updateKeys();
  };

  /**
   * @brief rebuild the keys and sort them, which is time consuming I guess.
   *        This function will be called in constructor.
   */
  void updateKeys() {
    keys.clear();
    // https://stackoverflow.com/questions/26281979/c-loop-through-map
    for (auto &[k, v] : m) {
      keys.push_back(k);
    }
    std::sort(keys.begin(), keys.end());
    max_key = *std::max_element(keys.begin(), keys.end());
  }

  T retrieve(int val) const {
    return retrieveByVal(keys, m, val);
  }
};

using tp = decltype(std::chrono::steady_clock::now());

class Instant {
  tp time;
  using duration_t = std::chrono::steady_clock::duration;

public:
  Instant() {
    this->time = std::chrono::steady_clock::now();
  }

  /// get the difference between now and the time Instant declared
  duration_t elapsed() const {
    const auto now      = std::chrono::steady_clock::now();
    const auto duration = now - this->time;
    return duration;
  }

  void reset() {
    const auto now = std::chrono::steady_clock::now();
    this->time     = now;
  }

  duration_t elapsed_and_reset() {
    const auto now      = std::chrono::steady_clock::now();
    const auto duration = now - this->time;
    this->time          = now;
    return duration;
  }

  [[nodiscard]] decltype(std::chrono::steady_clock::now()) count() const {
    return time;
  }
};

class ESPInstant {
  using duration_t   = std::chrono::duration<int64_t, std::micro>;
  using time_point_t = decltype(esp_timer_get_time());
  time_point_t time;

public:
  ESPInstant() {
    this->time = esp_timer_get_time();
  }

  duration_t elapsed() const {
    const auto now      = esp_timer_get_time();
    const auto diff     = now - this->time;
    const auto duration = duration_t{diff};
    return duration;
  }

  void reset() {
    const auto now = esp_timer_get_time();
    this->time     = now;
  }

  duration_t elapsed_and_reset() {
    const auto now      = esp_timer_get_time();
    const auto duration = now - this->time;
    this->time          = now;
    return duration_t{duration};
  }

  [[nodiscard]] time_point_t count() const {
    return time;
  }
};

namespace utils {

/**
 * @brief put hex string to out
 * @param out the output buffer
 * @param outSize the size of output buffer, will NOT include the null terminator
 * @param bytes the input bytes
 * @param size the size of input bytes
 * @return the length of output string
 * @note length of output string will not include the null terminator
 *       and the user should add it manually to the buffer, if null terminator is needed
 */
size_t sprintHex(char *out, size_t outSize, const uint8_t *bytes, size_t size);

std::string toHex(const uint8_t *bytes, size_t size);

#if __cplusplus >= 202002L
template <typename T>
concept is_intmax_t = std::is_same_v<T, decltype(std::ratio<1>::num) &>;
#endif

#if __cplusplus >= 202002L
template <typename T>
concept is_ratio = requires(T t) {
  { t.num } -> is_intmax_t;
  { t.den } -> is_intmax_t;
};
#endif

template <class Rep, class RI = std::ratio<1>>
#if __cplusplus >= 202002L
  requires is_ratio<RI> && std::is_arithmetic_v<Rep>
#endif
class length {
  Rep value;

public:
  using rep   = Rep;
  using ratio = RI;
  using Self  = length<Rep, RI>;
  constexpr Rep count() const {
    return value;
  }
  length() = default;
  explicit constexpr length(Rep rep) : value(rep) {}

  constexpr Self operator+=(const Self rhs) const {
    value += rhs.value;
    return *this;
  }

  constexpr Self operator-=(const Self rhs) const {
    value -= rhs.value;
    return *this;
  }

  constexpr Self operator+(Self rhs) const {
    return Self(value + rhs.value);
  }

  constexpr Self operator-(const Self rhs) const {
    return Self(value - rhs.value);
  }

  // Scalar operations

  template <class U>
#if __cplusplus >= 202002L
    requires std::is_arithmetic_v<U>
#endif
  constexpr Self operator*(U rhs) const {
    return Self(value * rhs);
  }

  template <class U>
#if __cplusplus >= 202002L
    requires std::is_arithmetic_v<U>
#endif
  constexpr Self operator/(U rhs) const {
    return Self(value / rhs);
  }

  // Comparison operators

  constexpr bool operator==(const Self rhs) const {
    return value == rhs.value;
  }

  constexpr bool operator!=(const Self rhs) const {
    return value != rhs.value;
  }

  constexpr bool operator<(const Self rhs) const {
    return value < rhs.value;
  }

  constexpr bool operator>(const Self rhs) const {
    return value > rhs.value;
  }

  constexpr bool operator<=(const Self rhs) const {
    return value <= rhs.value;
  }

  constexpr bool operator>=(const Self rhs) const {
    return value >= rhs.value;
  }
};

template <class FromLength, class ToLength,
          class RI = typename std::ratio_divide<typename FromLength::ratio, typename ToLength::ratio>::type>
struct __length_cast {
  constexpr ToLength operator()(const FromLength &fl) const {
    using _Ct = typename std::common_type<typename FromLength::rep, typename ToLength::rep, intmax_t>::type;
    return ToLength(static_cast<typename ToLength::rep>(
        static_cast<_Ct>(fl.count()) * static_cast<_Ct>(RI::num) / static_cast<_Ct>(RI::den)));
  };
};

template <class ToLength, class Rep, class RI>
#if __cplusplus >= 202002L
  requires std::is_arithmetic_v<Rep> && is_ratio<RI>
#endif
constexpr ToLength length_cast(const length<Rep, RI> &fl) {
  return __length_cast<length<Rep, RI>, ToLength>()(fl);
}

using picometer  = length<int, std::pico>;
using micrometer = length<int, std::micro>;
using milimeter  = length<int, std::milli>;
using centimeter = length<int, std::centi>;
using meter      = length<int, std::ratio<1>>;
using inch       = length<int, std::ratio<254, 10000>>;
using feet       = length<int, std::ratio<328, 1000>>;
using kilometer  = length<int, std::kilo>;

struct Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;

  Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
  Color(uint32_t rgb) {
    r = (rgb >> 16) & 0xff;
    g = (rgb >> 8) & 0xff;
    b = rgb & 0xff;
  }

  /**
   * @brief convert to uint32_t
   * @return implicit conversion. operator overloading.
   */
  operator uint32_t() const {
    return (r << 16) | (g << 8) | b;
  }
};
namespace Colors {
  const auto Red     = Color(0xff0000);
  const auto Green   = Color(0x00ff00);
  const auto Blue    = Color(0x0000ff);
  const auto White   = Color(0xffffff);
  const auto Cyan    = Color(0x00ffff);
  const auto Magenta = Color(0xff00ff);
  const auto Yellow  = Color(0xffff00);
  const auto Amber   = Color(0xffbf00);
  const auto Orange  = Color(0xff8000);
  const auto Purple  = Color(0x8000ff);
  const auto Pink    = Color(0xff0080);
  const auto Azure   = Color(0x0080ff);
}
};

#endif // HELLO_WORLD_UTILS_H
