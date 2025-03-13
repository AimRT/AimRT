// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <functional>
#include <source_location>
#include <string>
#include <thread>

#include "util/block_queue.h"
#include "util/exception.h"
#include "util/format.h"
#include "util/time_util.h"

#if __GLIBC__ == 2 && __GLIBC_MINOR__ < 30
  #include <sys/syscall.h>
  #define gettid() syscall(SYS_gettid)
#endif

namespace aimrt::common::util {

constexpr uint32_t kLogLevelTrace = 0;
constexpr uint32_t kLogLevelDebug = 1;
constexpr uint32_t kLogLevelInfo = 2;
constexpr uint32_t kLogLevelWarn = 3;
constexpr uint32_t kLogLevelError = 4;
constexpr uint32_t kLogLevelFatal = 5;

class SimpleLogger {
 public:
  static uint32_t GetLogLevel() { return 0; }

  static void Log(uint32_t lvl,
                  uint32_t line,
                  const char* file_name,
                  const char* function_name,
                  const char* log_data,
                  size_t log_data_size) {
    static constexpr std::string_view kLvlNameArray[] = {
        "Trace", "Debug", "Info", "Warn", "Error", "Fatal"};

    static constexpr uint32_t kLvlNameArraySize =
        sizeof(kLvlNameArray) / sizeof(kLvlNameArray[0]);
    lvl = lvl >= kLvlNameArraySize ? (kLvlNameArraySize - 1) : lvl;

#if defined(_WIN32)
    thread_local size_t tid(std::hash<std::thread::id>{}(std::this_thread::get_id()));
#else
    thread_local size_t tid(gettid());
#endif

    auto t = std::chrono::system_clock::now();
    std::string log_str = ::aimrt_fmt::format(
        "[{}.{:0>6}][{}][{}][{}:{} @{}]{}",
        GetTimeStr(std::chrono::system_clock::to_time_t(t)),
        (GetTimestampUs(t) % 1000000),
        kLvlNameArray[lvl],
        tid,
        file_name,
        line,
        function_name,
        std::string_view(log_data, log_data_size));

    fprintf(stderr, "%s\n", log_str.c_str());
  }
};

class SimpleAsyncLogger {
 public:
  SimpleAsyncLogger()
      : log_thread_(std::bind(&SimpleAsyncLogger::LogThread, this)) {}

  ~SimpleAsyncLogger() {
    queue_.Stop();
    if (log_thread_.joinable()) log_thread_.join();
  }

  SimpleAsyncLogger(const SimpleAsyncLogger&) = delete;
  SimpleAsyncLogger& operator=(const SimpleAsyncLogger&) = delete;

  static uint32_t GetLogLevel() { return 0; }

  void Log(uint32_t lvl,
           uint32_t line,
           const char* file_name,
           const char* function_name,
           const char* log_data,
           size_t log_data_size) const {
    static constexpr std::string_view kLvlNameArray[] = {
        "Trace", "Debug", "Info", "Warn", "Error", "Fatal"};

    static constexpr uint32_t kLvlNameArraySize =
        sizeof(kLvlNameArray) / sizeof(kLvlNameArray[0]);
    if (lvl >= kLvlNameArraySize) [[unlikely]]
      lvl = kLvlNameArraySize;

#if defined(_WIN32)
    thread_local size_t tid(std::hash<std::thread::id>{}(std::this_thread::get_id()));
#else
    thread_local size_t tid(gettid());
#endif

    auto t = std::chrono::system_clock::now();
    std::string log_str = ::aimrt_fmt::format(
        "[{}.{:0>6}][{}][{}][{}:{} @{}]{}",
        GetTimeStr(std::chrono::system_clock::to_time_t(t)),
        (GetTimestampUs(t) % 1000000),
        kLvlNameArray[lvl],
        tid,
        file_name,
        line,
        function_name,
        std::string_view(log_data, log_data_size));
    try {
      queue_.Enqueue(std::move(log_str));
    } catch (...) {
    }
  }

 private:
  void LogThread() {
    while (true) {
      try {
        auto log_str = queue_.Dequeue();
        fprintf(stderr, "%s\n", log_str.c_str());
      } catch (...) {
        break;
      }
    }
  }

  mutable BlockQueue<std::string> queue_;
  std::thread log_thread_;
};

struct LoggerWrapper {
  uint32_t GetLogLevel() const {
    return get_log_level_func();
  }

  void Log(uint32_t lvl,
           uint32_t line,
           const char* file_name,
           const char* function_name,
           const char* log_data,
           size_t log_data_size) const {
    log_func(lvl, line, file_name, function_name, log_data, log_data_size);
  }

  using GetLogLevelFunc = std::function<uint32_t(void)>;
  using LogFunc = std::function<void(uint32_t, uint32_t, const char*, const char*, const char*, size_t)>;

  GetLogLevelFunc get_log_level_func = SimpleLogger::GetLogLevel;
  LogFunc log_func = SimpleLogger::Log;
};

template <typename T>
concept LoggerType =
    requires(T t) {
      { t.GetLogLevel() } -> std::same_as<uint32_t>;
      { t.Log(0, 0, 0, nullptr, nullptr, nullptr, 0) };
    };

template <LoggerType Logger, typename... Args>
inline void LogImpl(const Logger& logger,
                    uint32_t lvl,
                    uint32_t line,
                    const char* file_name,
                    const char* function_name,
                    ::aimrt_fmt::format_string<Args...> fmt,
                    Args&&... args) {
  std::string log_str = ::aimrt_fmt::format(fmt, std::forward<Args>(args)...);
  logger.Log(lvl, line, file_name, function_name, log_str.c_str(), log_str.size());
}

}  // namespace aimrt::common::util

/// Log with the specified logger handle
#define AIMRT_HANDLE_LOG(__lgr__, __lvl__, __fmt__, ...)                        \
  do {                                                                          \
    const auto& __cur_lgr__ = __lgr__;                                          \
    if (__lvl__ >= __cur_lgr__.GetLogLevel()) {                                 \
      std::string __log_str__ = ::aimrt_fmt::format(__fmt__, ##__VA_ARGS__);    \
      constexpr auto __location__ = std::source_location::current();            \
      __cur_lgr__.Log(                                                          \
          __lvl__, __location__.line(), __location__.file_name(), __FUNCTION__, \
          __log_str__.c_str(), __log_str__.size());                             \
    }                                                                           \
  } while (0)

/// Log with the specified logger handle only once
#define AIMRT_HANDLE_LOG_ONCE(__lgr__, __lvl__, __fmt__, ...)     \
  do {                                                            \
    static bool __logged_line__ = false;                          \
    if (!__logged_line__) {                                       \
      __logged_line__ = true;                                     \
      AIMRT_HANDLE_LOG(__lgr__, __lvl__, __fmt__, ##__VA_ARGS__); \
    }                                                             \
  } while (0)

/// Log with the specified logger handle with cond
#define AIMRT_HANDLE_LOG_IF(__cond__, __lgr__, __lvl__, __fmt__, ...) \
  do {                                                                \
    if (__cond__) {                                                   \
      AIMRT_HANDLE_LOG(__lgr__, __lvl__, __fmt__, ##__VA_ARGS__);     \
    }                                                                 \
  } while (0)

/// Check and log with the specified logger handle
#define AIMRT_HANDLE_CHECK_LOG(__lgr__, __expr__, __lvl__, __fmt__, ...) \
  do {                                                                   \
    if (!(__expr__)) [[unlikely]] {                                      \
      AIMRT_HANDLE_LOG(__lgr__, __lvl__, __fmt__, ##__VA_ARGS__);        \
    }                                                                    \
  } while (0)

/// Log and throw with the specified logger handle
#define AIMRT_HANDLE_LOG_THROW(__lgr__, __lvl__, __fmt__, ...)             \
  do {                                                                     \
    std::string __log_str__ = ::aimrt_fmt::format(__fmt__, ##__VA_ARGS__); \
    const auto& __cur_lgr__ = __lgr__;                                     \
    if (__lvl__ >= __cur_lgr__.GetLogLevel()) {                            \
      constexpr auto __location__ = std::source_location::current();       \
      __cur_lgr__.Log(                                                     \
          __lvl__, __location__.line(), __location__.file_name(),          \
          __FUNCTION__, __log_str__.c_str(), __log_str__.size());          \
    }                                                                      \
    throw aimrt::common::util::AimRTException(std::move(__log_str__));     \
  } while (0)

/// Check and log and throw with the specified logger handle
#define AIMRT_HANDLE_CHECK_LOG_THROW(__lgr__, __expr__, __lvl__, __fmt__, ...) \
  do {                                                                         \
    if (!(__expr__)) [[unlikely]] {                                            \
      AIMRT_HANDLE_LOG_THROW(__lgr__, __lvl__, __fmt__, ##__VA_ARGS__);        \
    }                                                                          \
  } while (0)

#define AIMRT_HL_TRACE(__lgr__, __fmt__, ...) \
  AIMRT_HANDLE_LOG(__lgr__, aimrt::common::util::kLogLevelTrace, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_DEBUG(__lgr__, __fmt__, ...) \
  AIMRT_HANDLE_LOG(__lgr__, aimrt::common::util::kLogLevelDebug, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_INFO(__lgr__, __fmt__, ...) \
  AIMRT_HANDLE_LOG(__lgr__, aimrt::common::util::kLogLevelInfo, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_WARN(__lgr__, __fmt__, ...) \
  AIMRT_HANDLE_LOG(__lgr__, aimrt::common::util::kLogLevelWarn, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_ERROR(__lgr__, __fmt__, ...) \
  AIMRT_HANDLE_LOG(__lgr__, aimrt::common::util::kLogLevelError, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_FATAL(__lgr__, __fmt__, ...) \
  AIMRT_HANDLE_LOG(__lgr__, aimrt::common::util::kLogLevelFatal, __fmt__, ##__VA_ARGS__)

#define AIMRT_HL_CHECK_TRACE(__lgr__, __expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG(__lgr__, __expr__, aimrt::common::util::kLogLevelTrace, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_CHECK_DEBUG(__lgr__, __expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG(__lgr__, __expr__, aimrt::common::util::kLogLevelDebug, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_CHECK_INFO(__lgr__, __expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG(__lgr__, __expr__, aimrt::common::util::kLogLevelInfo, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_CHECK_WARN(__lgr__, __expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG(__lgr__, __expr__, aimrt::common::util::kLogLevelWarn, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_CHECK_ERROR(__lgr__, __expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG(__lgr__, __expr__, aimrt::common::util::kLogLevelError, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_CHECK_FATAL(__lgr__, __expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG(__lgr__, __expr__, aimrt::common::util::kLogLevelFatal, __fmt__, ##__VA_ARGS__)

#define AIMRT_HL_TRACE_THROW(__lgr__, __fmt__, ...) \
  AIMRT_HANDLE_LOG_THROW(__lgr__, aimrt::common::util::kLogLevelTrace, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_DEBUG_THROW(__lgr__, __fmt__, ...) \
  AIMRT_HANDLE_LOG_THROW(__lgr__, aimrt::common::util::kLogLevelDebug, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_INFO_THROW(__lgr__, __fmt__, ...) \
  AIMRT_HANDLE_LOG_THROW(__lgr__, aimrt::common::util::kLogLevelInfo, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_WARN_THROW(__lgr__, __fmt__, ...) \
  AIMRT_HANDLE_LOG_THROW(__lgr__, aimrt::common::util::kLogLevelWarn, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_ERROR_THROW(__lgr__, __fmt__, ...) \
  AIMRT_HANDLE_LOG_THROW(__lgr__, aimrt::common::util::kLogLevelError, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_FATAL_THROW(__lgr__, __fmt__, ...) \
  AIMRT_HANDLE_LOG_THROW(__lgr__, aimrt::common::util::kLogLevelFatal, __fmt__, ##__VA_ARGS__)

#define AIMRT_HL_CHECK_TRACE_THROW(__lgr__, __expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG_THROW(__lgr__, __expr__, aimrt::common::util::kLogLevelTrace, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_CHECK_DEBUG_THROW(__lgr__, __expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG_THROW(__lgr__, __expr__, aimrt::common::util::kLogLevelDebug, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_CHECK_INFO_THROW(__lgr__, __expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG_THROW(__lgr__, __expr__, aimrt::common::util::kLogLevelInfo, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_CHECK_WARN_THROW(__lgr__, __expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG_THROW(__lgr__, __expr__, aimrt::common::util::kLogLevelWarn, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_CHECK_ERROR_THROW(__lgr__, __expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG_THROW(__lgr__, __expr__, aimrt::common::util::kLogLevelError, __fmt__, ##__VA_ARGS__)
#define AIMRT_HL_CHECK_FATAL_THROW(__lgr__, __expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG_THROW(__lgr__, __expr__, aimrt::common::util::kLogLevelFatal, __fmt__, ##__VA_ARGS__)

/// Log with the default logger handle in current scope
#ifndef AIMRT_DEFAULT_LOGGER_HANDLE
  #define AIMRT_DEFAULT_LOGGER_HANDLE GetLogger()
#endif

#define AIMRT_TRACE(__fmt__, ...) \
  AIMRT_HANDLE_LOG(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelTrace, __fmt__, ##__VA_ARGS__)
#define AIMRT_DEBUG(__fmt__, ...) \
  AIMRT_HANDLE_LOG(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelDebug, __fmt__, ##__VA_ARGS__)
#define AIMRT_INFO(__fmt__, ...) \
  AIMRT_HANDLE_LOG(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelInfo, __fmt__, ##__VA_ARGS__)
#define AIMRT_WARN(__fmt__, ...) \
  AIMRT_HANDLE_LOG(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelWarn, __fmt__, ##__VA_ARGS__)
#define AIMRT_ERROR(__fmt__, ...) \
  AIMRT_HANDLE_LOG(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelError, __fmt__, ##__VA_ARGS__)
#define AIMRT_FATAL(__fmt__, ...) \
  AIMRT_HANDLE_LOG(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelFatal, __fmt__, ##__VA_ARGS__)

#define AIMRT_TRACE_ONCE(__fmt__, ...) \
  AIMRT_HANDLE_LOG_ONCE(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelTrace, __fmt__, ##__VA_ARGS__)
#define AIMRT_DEBUG_ONCE(__fmt__, ...) \
  AIMRT_HANDLE_LOG_ONCE(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelDebug, __fmt__, ##__VA_ARGS__)
#define AIMRT_INFO_ONCE(__fmt__, ...) \
  AIMRT_HANDLE_LOG_ONCE(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelInfo, __fmt__, ##__VA_ARGS__)
#define AIMRT_WARN_ONCE(__fmt__, ...) \
  AIMRT_HANDLE_LOG_ONCE(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelWarn, __fmt__, ##__VA_ARGS__)
#define AIMRT_ERROR_ONCE(__fmt__, ...) \
  AIMRT_HANDLE_LOG_ONCE(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelError, __fmt__, ##__VA_ARGS__)
#define AIMRT_FATAL_ONCE(__fmt__, ...) \
  AIMRT_HANDLE_LOG_ONCE(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelFatal, __fmt__, ##__VA_ARGS__)

#define AIMRT_TRACE_IF(__cond__, __fmt__, ...) \
  AIMRT_HANDLE_LOG_IF(__cond__, AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelTrace, __fmt__, ##__VA_ARGS__)
#define AIMRT_DEBUG_IF(__cond__, __fmt__, ...) \
  AIMRT_HANDLE_LOG_IF(__cond__, AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelDebug, __fmt__, ##__VA_ARGS__)
#define AIMRT_INFO_IF(__cond__, __fmt__, ...) \
  AIMRT_HANDLE_LOG_IF(__cond__, AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelInfo, __fmt__, ##__VA_ARGS__)
#define AIMRT_WARN_IF(__cond__, __fmt__, ...) \
  AIMRT_HANDLE_LOG_IF(__cond__, AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelWarn, __fmt__, ##__VA_ARGS__)
#define AIMRT_ERROR_IF(__cond__, __fmt__, ...) \
  AIMRT_HANDLE_LOG_IF(__cond__, AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelError, __fmt__, ##__VA_ARGS__)
#define AIMRT_FATAL_IF(__cond__, __fmt__, ...) \
  AIMRT_HANDLE_LOG_IF(__cond__, AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelFatal, __fmt__, ##__VA_ARGS__)

#define AIMRT_CHECK_TRACE(__expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG(AIMRT_DEFAULT_LOGGER_HANDLE, __expr__, aimrt::common::util::kLogLevelTrace, __fmt__, ##__VA_ARGS__)
#define AIMRT_CHECK_DEBUG(__expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG(AIMRT_DEFAULT_LOGGER_HANDLE, __expr__, aimrt::common::util::kLogLevelDebug, __fmt__, ##__VA_ARGS__)
#define AIMRT_CHECK_INFO(__expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG(AIMRT_DEFAULT_LOGGER_HANDLE, __expr__, aimrt::common::util::kLogLevelInfo, __fmt__, ##__VA_ARGS__)
#define AIMRT_CHECK_WARN(__expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG(AIMRT_DEFAULT_LOGGER_HANDLE, __expr__, aimrt::common::util::kLogLevelWarn, __fmt__, ##__VA_ARGS__)
#define AIMRT_CHECK_ERROR(__expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG(AIMRT_DEFAULT_LOGGER_HANDLE, __expr__, aimrt::common::util::kLogLevelError, __fmt__, ##__VA_ARGS__)
#define AIMRT_CHECK_FATAL(__expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG(AIMRT_DEFAULT_LOGGER_HANDLE, __expr__, aimrt::common::util::kLogLevelFatal, __fmt__, ##__VA_ARGS__)

#define AIMRT_TRACE_THROW(__fmt__, ...) \
  AIMRT_HANDLE_LOG_THROW(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelTrace, __fmt__, ##__VA_ARGS__)
#define AIMRT_DEBUG_THROW(__fmt__, ...) \
  AIMRT_HANDLE_LOG_THROW(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelDebug, __fmt__, ##__VA_ARGS__)
#define AIMRT_INFO_THROW(__fmt__, ...) \
  AIMRT_HANDLE_LOG_THROW(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelInfo, __fmt__, ##__VA_ARGS__)
#define AIMRT_WARN_THROW(__fmt__, ...) \
  AIMRT_HANDLE_LOG_THROW(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelWarn, __fmt__, ##__VA_ARGS__)
#define AIMRT_ERROR_THROW(__fmt__, ...) \
  AIMRT_HANDLE_LOG_THROW(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelError, __fmt__, ##__VA_ARGS__)
#define AIMRT_FATAL_THROW(__fmt__, ...) \
  AIMRT_HANDLE_LOG_THROW(AIMRT_DEFAULT_LOGGER_HANDLE, aimrt::common::util::kLogLevelFatal, __fmt__, ##__VA_ARGS__)

#define AIMRT_CHECK_TRACE_THROW(__expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG_THROW(AIMRT_DEFAULT_LOGGER_HANDLE, __expr__, aimrt::common::util::kLogLevelTrace, __fmt__, ##__VA_ARGS__)
#define AIMRT_CHECK_DEBUG_THROW(__expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG_THROW(AIMRT_DEFAULT_LOGGER_HANDLE, __expr__, aimrt::common::util::kLogLevelDebug, __fmt__, ##__VA_ARGS__)
#define AIMRT_CHECK_INFO_THROW(__expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG_THROW(AIMRT_DEFAULT_LOGGER_HANDLE, __expr__, aimrt::common::util::kLogLevelInfo, __fmt__, ##__VA_ARGS__)
#define AIMRT_CHECK_WARN_THROW(__expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG_THROW(AIMRT_DEFAULT_LOGGER_HANDLE, __expr__, aimrt::common::util::kLogLevelWarn, __fmt__, ##__VA_ARGS__)
#define AIMRT_CHECK_ERROR_THROW(__expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG_THROW(AIMRT_DEFAULT_LOGGER_HANDLE, __expr__, aimrt::common::util::kLogLevelError, __fmt__, ##__VA_ARGS__)
#define AIMRT_CHECK_FATAL_THROW(__expr__, __fmt__, ...) \
  AIMRT_HANDLE_CHECK_LOG_THROW(AIMRT_DEFAULT_LOGGER_HANDLE, __expr__, aimrt::common::util::kLogLevelFatal, __fmt__, ##__VA_ARGS__)
