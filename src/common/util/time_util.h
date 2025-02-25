// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <chrono>
#include <ctime>
#include <string>
#include <string_view>

namespace aimrt::common::util {

enum TimeConstant {
  kSecondPerMinute = 60,
  kMinutePerHour = 60,
  kSecondPerHour = 3600,
  kHourPerDay = 24,
  kSecondPerDay = 86400,
  kMsPerSecond = 1000,
  kUsPerMs = 1000,
  kUsPerSecond = 1000000,
  kDayPerMonth = 30,
  kDayPerWeek = 7,
  kMonthPerYear = 12,
};

/**
 * @brief Converts a time point to nanoseconds since epoch
 *
 * @tparam Clock Chrono clock type used for the time point
 * @param t Time point to convert
 * @return uint64_t nanosecond timestamp since epoch
 */
template <typename Clock>
  requires std::chrono::is_clock_v<Clock>
inline uint64_t GetTimestampNs(std::chrono::time_point<Clock> t) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}

/**
 * @brief Get the current timestamp in nanoseconds
 *
 * @tparam Clock Chrono clock type (defaults to std::chrono::system_clock)
 * @return uint64_t nanosecond timestamp of the current time
 */
template <typename Clock = std::chrono::system_clock>
  requires std::chrono::is_clock_v<Clock>
inline uint64_t GetCurTimestampNs() {
  return GetTimestampNs(Clock::now());
}

/**
 * @brief Converts a nanosecond timestamp to a time point of the specified clock type.
 *
 * @tparam Clock The clock type to use. Defaults to `std::chrono::system_clock`.
 * @param us The timestamp in nanoseconds.
 * @return A `std::chrono::time_point<Clock>` representing the timestamp.
 */
template <typename Clock = std::chrono::system_clock>
  requires std::chrono::is_clock_v<Clock>
inline const std::chrono::time_point<Clock> GetTimePointFromTimestampNs(uint64_t ns) {
  return std::chrono::time_point<Clock>(
      std::chrono::duration_cast<typename std::chrono::time_point<Clock>::duration>(
          std::chrono::nanoseconds(ns)));
}

/**
 * @brief Converts a time point to microseconds since epoch
 *
 * @tparam Clock Chrono clock type used for the time point
 * @param t Time point to convert
 * @return uint64_t microsecond timestamp since epoch
 */
template <typename Clock>
  requires std::chrono::is_clock_v<Clock>
inline uint64_t GetTimestampUs(std::chrono::time_point<Clock> t) {
  return std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
}

/**
 * @brief Get the current timestamp in microseconds
 *
 * @tparam Clock Chrono clock type (defaults to std::chrono::system_clock)
 * @return uint64_t microsecond timestamp of the current time
 */
template <typename Clock = std::chrono::system_clock>
  requires std::chrono::is_clock_v<Clock>
inline uint64_t GetCurTimestampUs() {
  return GetTimestampUs(Clock::now());
}

/**
 * @brief Converts a microsecond timestamp to a time point of the specified clock type.
 *
 * @tparam Clock The clock type to use. Defaults to `std::chrono::system_clock`.
 * @param us The timestamp in microseconds.
 * @return A `std::chrono::time_point<Clock>` representing the timestamp.
 */
template <typename Clock = std::chrono::system_clock>
  requires std::chrono::is_clock_v<Clock>
inline const std::chrono::time_point<Clock> GetTimePointFromTimestampUs(uint64_t us) {
  return std::chrono::time_point<Clock>(
      std::chrono::duration_cast<typename std::chrono::time_point<Clock>::duration>(
          std::chrono::microseconds(us)));
}

/**
 * @brief Converts a time point to milliseconds since epoch
 *
 * @tparam Clock Chrono clock type used for the time point
 * @param t Time point to convert
 * @return uint64_t millisecond timestamp since epoch
 */
template <typename Clock>
  requires std::chrono::is_clock_v<Clock>
inline uint64_t GetTimestampMs(std::chrono::time_point<Clock> t) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count();
}

/**
 * @brief Get the current timestamp in milliseconds
 *
 * @tparam Clock Chrono clock type (defaults to std::chrono::system_clock)
 * @return uint64_t millisecond timestamp of the current time
 */
template <typename Clock = std::chrono::system_clock>
  requires std::chrono::is_clock_v<Clock>
inline uint64_t GetCurTimestampMs() {
  return GetTimestampMs(Clock::now());
}

/**
 * @brief Converts a millisecond timestamp to a time point of the specified clock type.
 *
 * @tparam Clock The clock type to use. Defaults to `std::chrono::system_clock`.
 * @param us The timestamp in milliseconds.
 * @return A `std::chrono::time_point<Clock>` representing the timestamp.
 */
template <typename Clock = std::chrono::system_clock>
  requires std::chrono::is_clock_v<Clock>
inline const std::chrono::time_point<Clock> GetTimePointFromTimestampMs(uint64_t ms) {
  return std::chrono::time_point<Clock>(
      std::chrono::duration_cast<typename std::chrono::time_point<Clock>::duration>(
          std::chrono::milliseconds(ms)));
}

/**
 * @brief Converts a time point to seconds since epoch
 *
 * @tparam Clock Chrono clock type used for the time point
 * @param t Time point to convert
 * @return uint64_t second timestamp since epoch
 */
template <typename Clock>
  requires std::chrono::is_clock_v<Clock>
inline uint64_t GetTimestampSec(std::chrono::time_point<Clock> t) {
  return std::chrono::duration_cast<std::chrono::seconds>(t.time_since_epoch()).count();
}

/**
 * @brief Get the current timestamp in seconds
 *
 * @tparam Clock Chrono clock type (defaults to std::chrono::system_clock)
 * @return uint64_t second timestamp of the current time
 */
template <typename Clock = std::chrono::system_clock>
  requires std::chrono::is_clock_v<Clock>
inline uint64_t GetCurTimestampSec() {
  return GetTimestampSec(Clock::now());
}

/**
 * @brief Converts a second timestamp to a time point of the specified clock type.
 *
 * @tparam Clock The clock type to use. Defaults to `std::chrono::system_clock`.
 * @param us The timestamp in seconds.
 * @return A `std::chrono::time_point<Clock>` representing the timestamp.
 */
template <typename Clock = std::chrono::system_clock>
  requires std::chrono::is_clock_v<Clock>
inline const std::chrono::time_point<Clock> GetTimePointFromTimestampSec(uint64_t sec) {
  return std::chrono::time_point<Clock>(
      std::chrono::duration_cast<typename std::chrono::time_point<Clock>::duration>(
          std::chrono::seconds(sec)));
}

/**
 * @brief Get current time
 *
 * @return time_t
 */

inline time_t GetCurTimeT() {
  return std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
}

/**
 * @brief localtime
 *
 * @param t
 * @return struct tm
 */
inline struct tm TimeT2TmLocal(time_t t) {
  struct tm st;
#if defined(_WIN32)
  localtime_s(&st, &t);
#else
  localtime_r(&t, &st);
#endif
  return st;
}

/**
 * @brief gmtime
 *
 * @param t
 * @return struct tm
 */
inline struct tm TimeT2TmGm(time_t t) {
  struct tm st;
#if defined(_WIN32)
  gmtime_s(&st, &t);
#else
  gmtime_r(&t, &st);
#endif
  return st;
}

/**
 * @brief transform tm to string
 *
 * @param t
 * @return std::string_view
 */
inline std::string_view GetTimeStr(tm t) {
  thread_local char buf[20];  // 4+2+2+1+2+2+2+1
  snprintf(buf,
           sizeof(buf),
           "%04d-%02d-%02d %02d:%02d:%02d",
           (t.tm_year + 1900) % 10000u,
           (t.tm_mon + 1) % 100u,
           (t.tm_mday) % 100u,
           (t.tm_hour) % 100u,
           (t.tm_min) % 100u,
           (t.tm_sec) % 100u);
  return std::string_view(buf);
}

/**
 * @brief transform time_t to string
 *
 * @param t
 * @return std::string_view
 */
inline std::string_view GetTimeStr(time_t t) {
  return GetTimeStr(TimeT2TmLocal(t));
}

/**
 * @brief get current time
 *
 * @return struct tm
 */
inline struct tm GetCurTm() {
  return TimeT2TmLocal(GetCurTimeT());
}

/**
 * @brief get current time string
 *
 * @return std::string_view
 */
inline std::string_view GetCurTimeStr() {
  return GetTimeStr(GetCurTimeT());
}

/**
 * @brief determine if it is a leap year
 *
 * @param year
 * @return true
 * @return false
 */
inline bool IsLeapYear(uint32_t year) {
  return (year % 100) ? (year % 4 == 0) : (year % 400 == 0);
}

/**
 * @brief get current month day count
 *
 * @param year
 * @param month from 0 onwards, 0 represents January
 * @return uint32_t
 */
inline uint32_t GetMonthDayCount(uint32_t year, uint32_t month) {
  static constexpr uint32_t kMonthDayCount[] = {
      31,  // Jan
      28,  // Feb
      31,  // Mar
      30,  // Apr
      31,  // May
      30,  // Jun
      31,  // Jul
      31,  // Aug
      30,  // Sep
      31,  // Oct
      30,  // Nov
      31,  // Dec
  };
  return (month == 1 && IsLeapYear(year)) ? 29 : kMonthDayCount[month];
}

/**
 * @brief  Get local time zone
 *
 * @return int32_t timezone offset from UTC, unit is second
 */
inline int32_t GetLocalTimeZone() {
  time_t now = GetCurTimeT();
  struct tm gt, lt;

#if defined(_WIN32)
  gmtime_s(&gt, &now);
  localtime_s(&lt, &now);
#else
  gmtime_r(&now, &gt);
  localtime_r(&now, &lt);
#endif

  time_t gmt = mktime(&gt);
  time_t lmt = mktime(&lt);

  return (int32_t)difftime(lmt, gmt);
}

/**
 * @brief Determine if l_time is the next day of r_time
 *
 * @param l_time
 * @param r_time
 * @param time_zone timezone offset from UTC, unit is second
 * @return true
 * @return false
 */
inline bool IsPassDay(time_t l_time, time_t r_time, int32_t time_zone) {
  time_t l_day = static_cast<time_t>((l_time + time_zone) / kSecondPerDay);
  time_t r_day = static_cast<time_t>((r_time + time_zone) / kSecondPerDay);
  return (l_day > r_day);
}

/**
 * @brief Get the start time of the day of the given time point.
 *
 * @param t
 * @return time_t
 */
inline time_t GetDayStartTime(time_t t) {
  struct tm st = TimeT2TmLocal(t);

  st.tm_sec = 0;
  st.tm_min = 0;
  st.tm_hour = 0;

  return mktime(&st);
}

/**
 * @brief Get the day of the week, 1-7
 *
 * @param t
 * @return uint32_t 1~7
 */
inline uint32_t GetWeekDay(time_t t) {
  struct tm st = TimeT2TmLocal(t);
  return (st.tm_wday) ? (st.tm_wday) : 7;
}

/**
 * @brief Get the start time of the week of the given time point.
 *
 * @param t
 * @return uint32_t
 */
inline uint32_t GetWeekStartTime(time_t t) {
  return GetDayStartTime(t - (GetWeekDay(t) - 1) * kSecondPerDay);
}

/**
 * @brief Calculates the number of days between two timestamps.
 *
 * @param l_time
 * @param r_time
 * @param time_zone
 * @return int32_t
 */
inline int32_t GetDayCount(time_t l_time, time_t r_time, int32_t time_zone) {
  return (l_time + time_zone - r_time) / kSecondPerDay;
}

/**
 * @brief Get week day string
 *
 * @param t
 * @return std::string_view
 */
inline std::string_view GetWeekDayStr(time_t t) {
  static constexpr std::string_view kWeekDays[] = {
      "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
  struct tm st = TimeT2TmLocal(t);
  return kWeekDays[st.tm_wday];
}

/**
 * @brief Get week-short day string
 *
 * @param t
 * @return std::string_view
 */
inline std::string_view GetWeekDayStrShort(time_t t) {
  static constexpr std::string_view kWeekDaysShort[] = {
      "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  struct tm st = TimeT2TmLocal(t);
  return kWeekDaysShort[st.tm_wday];
}

/**
 * @brief Get year string (4 digits)
 *
 * @param t timestamp
 * @return std::string_view Format like "2024"
 */
inline std::string_view GetYearStr(time_t t) {
  thread_local char buf[5];
  auto tm = TimeT2TmLocal(t);
  snprintf(buf, sizeof(buf), "%04d", (tm.tm_year + 1900) % 10000u);
  return std::string_view(buf);
}

/**
 * @brief Get month string (2 digits)
 *
 * @param t  timestamp
 * @return std::string_view Format like "03"
 */
inline std::string_view GetMonthStr(time_t t) {
  thread_local char buf[3];
  auto tm = TimeT2TmLocal(t);
  snprintf(buf, sizeof(buf), "%02d", (tm.tm_mon + 1) % 100u);
  return std::string_view(buf);
}

/**
 * @brief Get day string (2 digits)
 *
 * @param t timestamp
 * @return std::string_view Format like "15"
 */
inline std::string_view GetDayStr(time_t t) {
  thread_local char buf[3];
  auto tm = TimeT2TmLocal(t);
  snprintf(buf, sizeof(buf), "%02d", tm.tm_mday % 100u);
  return std::string_view(buf);
}

/**
 * @brief Get hour string (2 digits, 24-hour format)
 *
 * @param t timestamp
 * @return std::string_view Format like "14"
 */
inline std::string_view GetHourStr(time_t t) {
  thread_local char buf[3];
  auto tm = TimeT2TmLocal(t);
  snprintf(buf, sizeof(buf), "%02d", tm.tm_hour % 100u);
  return std::string_view(buf);
}

/**
 * @brief Get minute string (2 digits)
 *
 * @param t timestamp
 * @return std::string_view Format like "30"
 */
inline std::string_view GetMinuteStr(time_t t) {
  thread_local char buf[3];
  auto tm = TimeT2TmLocal(t);
  snprintf(buf, sizeof(buf), "%02d", tm.tm_min % 100u);
  return std::string_view(buf);
}

/**
 * @brief Get second string (2 digits)
 *
 * @param t timestamp
 * @return std::string_view Format like "45"
 */
inline std::string_view GetSecondStr(time_t t) {
  thread_local char buf[3];
  auto tm = TimeT2TmLocal(t);
  snprintf(buf, sizeof(buf), "%02d", tm.tm_sec % 100u);
  return std::string_view(buf);
}

/**
 * @brief Get complete date string
 *
 * @param t timestamp
 * @return std::string_view Format like "2024-03-15"
 */
inline std::string_view GetDateStr(time_t t) {
  thread_local char buf[11];
  auto tm = TimeT2TmLocal(t);
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d",
           (tm.tm_year + 1900) % 10000u,
           (tm.tm_mon + 1) % 100u,
           tm.tm_mday % 100u);
  return std::string_view(buf);
}

/**
 * @brief Get complete time string
 *
 * @param t timestamp
 * @return std::string_view Format like "14:30:45"
 */
inline std::string_view GetClockStr(time_t t) {
  thread_local char buf[9];
  auto tm = TimeT2TmLocal(t);
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
           tm.tm_hour % 100u,
           tm.tm_min % 100u,
           tm.tm_sec % 100u);
  return std::string_view(buf);
}

}  // namespace aimrt::common::util
