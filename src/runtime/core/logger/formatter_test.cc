// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "formatter.h"
#include <gtest/gtest.h>

namespace aimrt::runtime::core::logger {

// Test the GetRemappedFuncName function
TEST(FORMATTER_TEST, Format_test) {
  auto date = std::chrono::year(2024) / std::chrono::month(10) / std::chrono::day(1);
  auto time = std::chrono::hours(10) +
              std::chrono::minutes(10) +
              std::chrono::seconds(10) +
              std::chrono::microseconds(123456);

  // // 2024-10-01 10:10:10.123456
  auto datatime = std::chrono::sys_days(date) + time - std::chrono::seconds(common::util::GetLocalTimeZone());

  const char* test_msg = "test log message";
  LogDataWrapper log_data_wrapper{
      .module_name = "test_module",
      .thread_id = 1234,
      .t = datatime,
      .lvl = AIMRT_LOG_LEVEL_INFO,
      .line = 20,
      .column = 10,
      .file_name = "XX/YY/ZZ/test_file.cpp",
      .function_name = "test_function",
      .log_data = test_msg,
      .log_data_size = strlen(test_msg)

  };

  LogFormatter formatter;
  std::string actual_output;
  std::string expected_output;

  // Test the default pattern
  formatter.SetPattern("[%c.%f][%l][%t][%n][%g:%R:%C @%F]%v");
  expected_output = "[2024-10-01 10:10:10.123456][Info][1234][test_module][XX/YY/ZZ/test_file.cpp:20:10 @test_function]test log message";
  actual_output = formatter.Format(log_data_wrapper);
  EXPECT_EQ(actual_output, expected_output);

  // Test dividing the time into year, month, day, hour, minute, second, and weekday
  actual_output.clear();
  formatter.SetPattern("[%Y/%m/%d %H:%M:%S.%f][%A][%a]%v");
  expected_output = "[2024/10/01 10:10:10.123456][Tuesday][Tue]test log message";
  actual_output = formatter.Format(log_data_wrapper);
  EXPECT_EQ(actual_output, expected_output);

  // Test custom pattern
  actual_output.clear();
  formatter.SetPattern("%%123456%%%q!@#$%^&*()+-*/12345678%%");
  expected_output = "%123456%q!@#$^&*()+-*/12345678%";
  actual_output = formatter.Format(log_data_wrapper);
  EXPECT_EQ(actual_output, expected_output);

  // Test blank pattern
  actual_output.clear();
  EXPECT_ANY_THROW(formatter.SetPattern(""));
}

}  // namespace aimrt::runtime::core::logger