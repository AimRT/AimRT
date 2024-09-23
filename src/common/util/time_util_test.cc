// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "util/time_util.h"

namespace aimrt::common::util {

TEST(TIME_UTIL_TEST, Timestamp_test) {
  using namespace std::chrono;

  auto tp = system_clock::now();

  uint64_t ns_timestamp = GetTimestampNs(tp);
  auto ns_tp = GetTimePointFromTimestampNs(ns_timestamp);
  EXPECT_EQ(duration_cast<nanoseconds>(ns_tp.time_since_epoch()).count(), ns_timestamp);

  uint64_t us_timestamp = GetTimestampUs(tp);
  auto us_tp = GetTimePointFromTimestampUs(us_timestamp);
  EXPECT_EQ(duration_cast<microseconds>(us_tp.time_since_epoch()).count(), us_timestamp);

  uint64_t ms_timestamp = GetTimestampMs(tp);
  auto ms_tp = GetTimePointFromTimestampMs(ms_timestamp);
  EXPECT_EQ(duration_cast<milliseconds>(ms_tp.time_since_epoch()).count(), ms_timestamp);

  uint64_t sec_timestamp = GetTimestampSec(tp);
  auto sec_tp = GetTimePointFromTimestampSec(sec_timestamp);
  EXPECT_EQ(duration_cast<seconds>(sec_tp.time_since_epoch()).count(), sec_timestamp);
}

TEST(TIME_UTIL_TEST, IsLeapYear) {
  EXPECT_TRUE(IsLeapYear(2000));
  EXPECT_FALSE(IsLeapYear(2001));
  EXPECT_FALSE(IsLeapYear(1900));
}

TEST(TIME_UTIL_TEST, GetMonthDayCount) {
  EXPECT_EQ(GetMonthDayCount(2000, 0), 31);
  EXPECT_EQ(GetMonthDayCount(2000, 1), 29);
  EXPECT_EQ(GetMonthDayCount(2000, 2), 31);
  EXPECT_EQ(GetMonthDayCount(2000, 3), 30);
  EXPECT_EQ(GetMonthDayCount(2000, 4), 31);
  EXPECT_EQ(GetMonthDayCount(2000, 5), 30);
  EXPECT_EQ(GetMonthDayCount(2000, 6), 31);
  EXPECT_EQ(GetMonthDayCount(2000, 7), 31);
  EXPECT_EQ(GetMonthDayCount(2000, 8), 30);
  EXPECT_EQ(GetMonthDayCount(2000, 9), 31);
  EXPECT_EQ(GetMonthDayCount(2000, 10), 30);
  EXPECT_EQ(GetMonthDayCount(2000, 11), 31);

  EXPECT_EQ(GetMonthDayCount(2001, 1), 28);
}

}  // namespace aimrt::common::util
