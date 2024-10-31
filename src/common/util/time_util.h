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
 * @brief 获取纳秒时间戳
 *
 * @param t 时间点
 * @return uint64_t 纳秒时间戳
 */
inline uint64_t GetTimestampNs(std::chrono::system_clock::time_point t) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}

/**
 * @brief 获取当前时间纳秒时间戳
 *
 * @return uint64_t 当前时间纳秒时间戳
 */
inline uint64_t GetCurTimestampNs() {
  return GetTimestampNs(std::chrono::system_clock::now());
}

/**
 * @brief 从纳秒时间戳转换为system_clock::time_point
 *
 * @param ns 纳秒时间戳
 * @return const std::chrono::system_clock::time_point
 */
inline const std::chrono::system_clock::time_point GetTimePointFromTimestampNs(uint64_t ns) {
  return std::chrono::system_clock::time_point(
      std::chrono::duration_cast<std::chrono::system_clock::time_point::duration>(
          std::chrono::nanoseconds(ns)));
}

/**
 * @brief 获取微秒时间戳
 *
 * @param t 时间点
 * @return uint64_t 微秒时间戳
 */
inline uint64_t GetTimestampUs(std::chrono::system_clock::time_point t) {
  return std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
}

/**
 * @brief 获取当前时间微秒时间戳
 *
 * @return uint64_t 当前时间微秒时间戳
 */
inline uint64_t GetCurTimestampUs() {
  return GetTimestampUs(std::chrono::system_clock::now());
}

/**
 * @brief 从微秒时间戳转换为system_clock::time_point
 *
 * @param us 微秒时间戳
 * @return const std::chrono::system_clock::time_point
 */
inline const std::chrono::system_clock::time_point GetTimePointFromTimestampUs(uint64_t us) {
  return std::chrono::system_clock::time_point(
      std::chrono::duration_cast<std::chrono::system_clock::time_point::duration>(
          std::chrono::microseconds(us)));
}

/**
 * @brief 获取毫秒时间戳
 *
 * @param t 时间点
 * @return uint64_t 毫秒时间戳
 */
inline uint64_t GetTimestampMs(std::chrono::system_clock::time_point t) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count();
}

/**
 * @brief 获取当前时间毫秒时间戳
 *
 * @return uint64_t 当前时间毫秒时间戳
 */
inline uint64_t GetCurTimestampMs() {
  return GetTimestampMs(std::chrono::system_clock::now());
}

/**
 * @brief 从毫秒时间戳转换为system_clock::time_point
 *
 * @param ms 毫秒时间戳
 * @return const std::chrono::system_clock::time_point
 */
inline const std::chrono::system_clock::time_point GetTimePointFromTimestampMs(uint64_t ms) {
  return std::chrono::system_clock::time_point(
      std::chrono::duration_cast<std::chrono::system_clock::time_point::duration>(
          std::chrono::milliseconds(ms)));
}

/**
 * @brief 获取秒时间戳
 *
 * @param t 时间点
 * @return uint64_t 秒时间戳
 */
inline uint64_t GetTimestampSec(std::chrono::system_clock::time_point t) {
  return std::chrono::duration_cast<std::chrono::seconds>(t.time_since_epoch()).count();
}

/**
 * @brief 获取当前时间秒时间戳
 *
 * @return uint64_t 当前时间秒时间戳
 */
inline uint64_t GetCurTimestampSec() {
  return GetTimestampSec(std::chrono::system_clock::now());
}

/**
 * @brief 从秒时间戳转换为system_clock::time_point
 *
 * @param sec 秒时间戳
 * @return const std::chrono::system_clock::time_point
 */
inline const std::chrono::system_clock::time_point GetTimePointFromTimestampSec(uint64_t sec) {
  return std::chrono::system_clock::time_point(
      std::chrono::duration_cast<std::chrono::system_clock::time_point::duration>(
          std::chrono::seconds(sec)));
}

/**
 * @brief 获取当前时间
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
 * @brief 时间转字符串
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
 * @brief 时间转字符串
 *
 * @param t
 * @return std::string_view
 */
inline std::string_view GetTimeStr(time_t t) {
  return GetTimeStr(TimeT2TmLocal(t));
}

/**
 * @brief 获取当前时间
 *
 * @return struct tm
 */
inline struct tm GetCurTm() {
  return TimeT2TmLocal(GetCurTimeT());
}

/**
 * @brief 获取当前时间字符串
 *
 * @return std::string_view
 */
inline std::string_view GetCurTimeStr() {
  return GetTimeStr(GetCurTimeT());
}

/**
 * @brief 判断是否为闰年
 *
 * @param year
 * @return true
 * @return false
 */
inline bool IsLeapYear(uint32_t year) {
  return (year % 100) ? (year % 4 == 0) : (year % 400 == 0);
}

/**
 * @brief 获取当月天数
 *
 * @param year 年份
 * @param month 从0开始，0代表1月
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
 * @brief 获取所在时区
 *
 * @return int32_t 时区，单位s
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
 * @brief 判断l_time是否在r_time后一天
 *
 * @param l_time
 * @param r_time
 * @param time_zone 时区，单位s
 * @return true
 * @return false
 */
inline bool IsPassDay(time_t l_time, time_t r_time, int32_t time_zone) {
  time_t l_day = static_cast<time_t>((l_time + time_zone) / kSecondPerDay);
  time_t r_day = static_cast<time_t>((r_time + time_zone) / kSecondPerDay);
  return (l_day > r_day);
}

/**
 * @brief 获取一天开始的时间点
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
 * @brief 获取星期几，1~7
 *
 * @param t
 * @return uint32_t 1~7
 */
inline uint32_t GetWeekDay(time_t t) {
  struct tm st = TimeT2TmLocal(t);
  return (st.tm_wday) ? (st.tm_wday) : 7;
}

/**
 * @brief 获取当周周一的开始时间点
 *
 * @param t
 * @return uint32_t
 */
inline uint32_t GetWeekStartTime(time_t t) {
  return GetDayStartTime(t - (GetWeekDay(t) - 1) * kSecondPerDay);
}

/**
 * @brief 计算两个时间之间日期差
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
 * @brief get week day string
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
 * @brief get week-short day string
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
