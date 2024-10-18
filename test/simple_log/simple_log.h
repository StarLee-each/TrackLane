//
// Created by Kurosu Chan on 2023/10/16.
//

#ifndef NANOPB_TEST_SIMPLE_LOG_H
#define NANOPB_TEST_SIMPLE_LOG_H
/**
 *
 * Why do I need macro here?
 *
 * The initial implementation was done with C++ template and variadic parameter.
 * However `vprintf` would eats up too much memory. I could only use `printf` (which seems weird since `printf`
 * should have called `vprintf` but due to some compiler magic the `printf` eats 466 bytes and the `vprintf`
 * would eats several KBs)
 *
 * The unavailability of `vprintf` means variadic parameter could not be used. One has to pass the arguments directly
 * to `printf`. Ugly indeed...
 *
 */

#ifndef SIMPLE_LOG
#define SIMPLE_LOG
#endif

namespace simple_log {
// somehow the Level is ordered
// the lower the level, the more verbose
    enum class Level {
        Trace = 0,
        Debug,
        Info,
        Warn,
        Error,
        Off,
    };
    constexpr auto level_ = Level::Debug;
    consteval bool is_enabled(Level level) {
        return level >= level_;
    }
    consteval bool is_debug() {
        return is_enabled(Level::Debug);
    }

    consteval bool is_trace() {
        return is_enabled(Level::Trace);
    }

    consteval bool is_info() {
        return is_enabled(Level::Info);
    }

    /**
     * @brief print the prefix of the log in format `[symbol][tag][time][file_name:line_number]`
     * @param level `simple_log::Level` the level of the log
     * @param symbol `char` the symbol of the log
     * @param [in]tag `const char*` the tag of the log
     * @param [in]file_name `const char*` the file name of the log
     * @param line_number `int` the line number of the log
     * @ref <print_prefix>"print_prefix"
     * @sa LOG_ME
     * @sa LOG_NO_LF
     * @sa LOG_WITH_TAIL
     */
    void print_prefix(char symbol, const char *tag, const char *file_name, int line_number);
    /**
     * @brief init the clock for the logger
     */
    void init();
}

/**
 * @brief log with line break and file info (file name and line number)
 * @param tail <expression> the expression to be evaluated after the log. It could be <del>IIFE</del> (brace-enclosed block is more suitable here) or a function call.
 * @copydoc LOG_ME
 * @ref <LOG_WTH_TAIL>"LOG_WITH_TAIL"
 */
#define LOG_WITH_TAIL(tail, level, symbol, tag, file_name, line_number, fmt, ...) \
  do {                                                                            \
    if constexpr ((level) < (simple_log::level_)) {                                  \
      break;                                                                      \
    }                                                                             \
    simple_log::print_prefix(symbol, tag, file_name, line_number);                   \
    printf(fmt, ##__VA_ARGS__);                                                   \
    do {                                                                          \
      tail;                                                                       \
    } while (0);                                                                  \
    putchar('\n');                                                                \
  } while (0)

#define LOGI_WITH_TAIL(tail, tag, fmt, ...) LOG_WITH_TAIL(tail, simple_log::Level::Info, 'I', tag, nullptr, 0, fmt, ##__VA_ARGS__)
#define LOGW_WITH_TAIL(tail, tag, fmt, ...) LOG_WITH_TAIL(tail, simple_log::Level::Warn, 'W', tag, nullptr, 0, fmt, ##__VA_ARGS__)
#define LOGE_WITH_TAIL(tail, tag, fmt, ...) LOG_WITH_TAIL(tail, simple_log::Level::Error, 'E', tag, nullptr, 0, fmt, ##__VA_ARGS__)
#define LOGD_WITH_TAIL(tail, tag, fmt, ...) LOG_WITH_TAIL(tail, simple_log::Level::Debug, 'D', tag, nullptr, 0, fmt, ##__VA_ARGS__)
#define LOGT_WITH_TAIL(tail, tag, fmt, ...) LOG_WITH_TAIL(tail, simple_log::Level::Trace, 'T', tag, nullptr, 0, fmt, ##__VA_ARGS__)

/**
 * @brief log without line break and file info
 * @copydoc LOG_ME
 * @ref <LOG_NO_LF>"LOG_NO_LF"
 */
#define LOG_NO_LF(level, symbol, tag, file_name, line_number, fmt, ...) \
  do {                                                                  \
    if constexpr ((level) < (simple_log::level_)) {                        \
      break;                                                            \
    }                                                                   \
    simple_log::print_prefix(symbol, tag, file_name, line_number);         \
    printf(fmt, ##__VA_ARGS__);                                         \
  } while (0)

/// log info without line break and file info
#define LOGI_NO_LF(tag, fmt, ...) LOG_NO_LF(simple_log::Level::Info, 'I', tag, nullptr, 0, fmt, ##__VA_ARGS__)
/// log warn without line break and file info
#define LOGW_NO_LF(tag, fmt, ...) LOG_NO_LF(simple_log::Level::Warn, 'W', tag, nullptr, 0, fmt, ##__VA_ARGS__)
/// log error without line break and file info
#define LOGE_NO_LF(tag, fmt, ...) LOG_NO_LF(simple_log::Level::Error, 'E', tag, nullptr, 0, fmt, ##__VA_ARGS__)
/// log debug without line break and file info
#define LOGD_NO_LF(tag, fmt, ...) LOG_NO_LF(simple_log::Level::Debug, 'D', tag, nullptr, 0, fmt, ##__VA_ARGS__)
/// log trace without line break and file info
#define LOGT_NO_LF(tag, fmt, ...) LOG_NO_LF(simple_log::Level::Trace, 'T', tag, nullptr, 0, fmt, ##__VA_ARGS__)

/**
 * @brief log with line break and file info (file name and line number)
 * @param level `simple_log::Level` the level of the log
 * @param symbol `char` the symbol of the log
 * @param [in]tag `const char*` the tag of the log
 * @param [in]file_name `const char*` the file name of the log
 * @param line_number `int` the line number of the log
 * @param [in]fmt `const char*` the format string of the log
 * @param [in]... variadic parameters
 * @ref <LOG_ME>"LOG_ME"
 * @sa simple_log::print_prefix
 */
#define LOG_ME(level, symbol, tag, file_name, line_number, fmt, ...) \
  do {                                                               \
    if constexpr ((level) < (simple_log::level_)) {                     \
      break;                                                         \
    }                                                                \
    simple_log::print_prefix(symbol, tag, file_name, line_number);      \
    printf(fmt, ##__VA_ARGS__);                                      \
    putchar('\n');                                                   \
  } while (0)

// https://stackoverflow.com/questions/8487986/file-macro-shows-full-path
// use __FILE_NAME__ instead of __FILE__ to get rid of the absolute path

/// log info with line break and file info
#define LOG_I(tag, fmt, ...) LOG_ME(simple_log::Level::Info, 'I', tag, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)
/// log warn with line break and file info
#define LOG_W(tag, fmt, ...) LOG_ME(simple_log::Level::Warn, 'W', tag, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)
/// log error with line break and file info
#define LOG_E(tag, fmt, ...) LOG_ME(simple_log::Level::Error, 'E', tag, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)
/// log debug with line break and file info
#define LOG_D(tag, fmt, ...) LOG_ME(simple_log::Level::Debug, 'D', tag, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)
/// log trace with line break and file info
#define LOG_T(tag, fmt, ...) LOG_ME(simple_log::Level::Trace, 'T', tag, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)

/// log info with line break but without file info
#define LOGI(tag, fmt, ...) LOG_ME(simple_log::Level::Info, 'I', tag, nullptr, 0, fmt, ##__VA_ARGS__)
/// log warn with line break but without file info
#define LOGW(tag, fmt, ...) LOG_ME(simple_log::Level::Warn, 'W', tag, nullptr, 0, fmt, ##__VA_ARGS__)
/// log error with line break but without file info
#define LOGE(tag, fmt, ...) LOG_ME(simple_log::Level::Error, 'E', tag, nullptr, 0, fmt, ##__VA_ARGS__)
/// log debug with line break but without file info
#define LOGD(tag, fmt, ...) LOG_ME(simple_log::Level::Debug, 'D', tag, nullptr, 0, fmt, ##__VA_ARGS__)
/// log trace with line break but without file info
#define LOGT(tag, fmt, ...) LOG_ME(simple_log::Level::Trace, 'T', tag, nullptr, 0, fmt, ##__VA_ARGS__)

#include "simple_log_utils.h"

#endif //NANOPB_TEST_SIMPLE_LOG_H
