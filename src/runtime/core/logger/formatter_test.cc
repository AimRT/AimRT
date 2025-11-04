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
              std::chrono::microseconds(12345);

  // // 2024-10-01 10:10:10.012345
  auto datatime = std::chrono::sys_days(date) + time - std::chrono::seconds(common::util::GetLocalTimeZone());

  const char* test_msg = "test log message";
#ifdef _WIN32
  const char* file_path = "XX\\YY\\ZZ\\test_file.cpp";
#else
  const char* file_path = "XX/YY/ZZ/test_file.cpp";
#endif

  LogDataWrapper log_data_wrapper{
      .module_name = "test_module",
      .thread_id = 1234,
      .t = datatime,
      .lvl = AIMRT_LOG_LEVEL_INFO,
      .line = 20,
      .file_name = file_path,
      .function_name = "test_function",
      .log_data = test_msg,
      .log_data_size = strlen(test_msg)

  };

  LogFormatter formatter;
  std::string actual_output;
  std::string expected_output;

  // Test the default pattern
  formatter.SetPattern("[%c.%f][%l][%t][%n][%g:%R @%F]%v");
#ifdef _WIN32
  expected_output = "[2024-10-01 10:10:10.012345][Info][1234][test_module][XX\\YY\\ZZ\\test_file.cpp:20 @test_function]test log message";
#else
  expected_output = "[2024-10-01 10:10:10.012345][Info][1234][test_module][XX/YY/ZZ/test_file.cpp:20 @test_function]test log message";
#endif
  actual_output = formatter.Format(log_data_wrapper);
  EXPECT_EQ(actual_output, expected_output);

  // Test dividing the time into year, month, day, hour, minute, second, and weekday
  actual_output.clear();
  formatter.SetPattern("[%Y/%m/%d %H:%M:%S.%f][%A][%a]%v");
  expected_output = "[2024/10/01 10:10:10.012345][Tuesday][Tue]test log message";
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

// Test for short filename format (%G)
TEST(FORMATTER_SHORT_FILENAME_TEST, Format_short_filename_test) {
  auto date = std::chrono::year(2024) / std::chrono::month(10) / std::chrono::day(1);
  auto time = std::chrono::hours(10) +
              std::chrono::minutes(10) +
              std::chrono::seconds(10) +
              std::chrono::microseconds(123456);

  auto datatime = std::chrono::sys_days(date) + time - std::chrono::seconds(common::util::GetLocalTimeZone());

  const char* test_msg = "test log message";
#ifdef _WIN32
  const char* file_path = "XX\\YY\\ZZ\\test_file.cpp";
#else
  const char* file_path = "XX/YY/ZZ/test_file.cpp";
#endif

  LogDataWrapper log_data_wrapper{
      .module_name = "test_module",
      .thread_id = 1234,
      .t = datatime,
      .lvl = AIMRT_LOG_LEVEL_INFO,
      .line = 20,
      .file_name = file_path,
      .function_name = "test_function",
      .log_data = test_msg,
      .log_data_size = std::string_view(test_msg).size()};

  LogFormatter formatter;
  std::string actual_output;
  std::string expected_output;

  // Test short filename format - should extract only "test_file.cpp"
  formatter.SetPattern("[%c.%f][%l][%t][%n][%G:%R @%F]%v");
  expected_output = "[2024-10-01 10:10:10.123456][Info][1234][test_module][test_file.cpp:20 @test_function]test log message";
  actual_output = formatter.Format(log_data_wrapper);
  EXPECT_EQ(actual_output, expected_output);
}

// Test for microsecond with full value
TEST(FORMATTER_MICROSECOND_TEST_FULL, Format_microsecond_test_full) {
  auto date = std::chrono::year(2024) / std::chrono::month(10) / std::chrono::day(1);
  auto time = std::chrono::hours(10) +
              std::chrono::minutes(10) +
              std::chrono::seconds(10) +
              std::chrono::microseconds(123456);

  // // 2024-10-01 10:10:10.123456
  auto datatime = std::chrono::sys_days(date) + time - std::chrono::seconds(common::util::GetLocalTimeZone());

  const char* test_msg = "test log message";
#ifdef _WIN32
  const char* file_path = "XX\\YY\\ZZ\\test_file.cpp";
#else
  const char* file_path = "XX/YY/ZZ/test_file.cpp";
#endif

  LogDataWrapper log_data_wrapper{
      .module_name = "test_module",
      .thread_id = 1234,
      .t = datatime,
      .lvl = AIMRT_LOG_LEVEL_INFO,
      .line = 20,
      .file_name = file_path,
      .function_name = "test_function",
      .log_data = test_msg,
      .log_data_size = strlen(test_msg)

  };

  LogFormatter formatter;
  std::string actual_output;
  std::string expected_output;

  formatter.SetPattern("[%c.%f][%l][%t][%n][%g:%R @%F]%v");
#ifdef _WIN32
  expected_output = "[2024-10-01 10:10:10.123456][Info][1234][test_module][XX\\YY\\ZZ\\test_file.cpp:20 @test_function]test log message";
#else
  expected_output = "[2024-10-01 10:10:10.123456][Info][1234][test_module][XX/YY/ZZ/test_file.cpp:20 @test_function]test log message";
#endif
  actual_output = formatter.Format(log_data_wrapper);
  EXPECT_EQ(actual_output, expected_output);
}

// Test for microsecond with zero value
TEST(FORMATTER_MICROSECOND_TEST_ZERO, Format_microsecond_test_zero) {
  auto date = std::chrono::year(2024) / std::chrono::month(10) / std::chrono::day(1);
  auto time = std::chrono::hours(10) +
              std::chrono::minutes(10) +
              std::chrono::seconds(10) +
              std::chrono::microseconds(0);

  // // 2024-10-01 10:10:10.000000
  auto datatime = std::chrono::sys_days(date) + time - std::chrono::seconds(common::util::GetLocalTimeZone());

  const char* test_msg = "test log message";
#ifdef _WIN32
  const char* file_path = "XX\\YY\\ZZ\\test_file.cpp";
#else
  const char* file_path = "XX/YY/ZZ/test_file.cpp";
#endif

  LogDataWrapper log_data_wrapper{
      .module_name = "test_module",
      .thread_id = 1234,
      .t = datatime,
      .lvl = AIMRT_LOG_LEVEL_INFO,
      .line = 20,
      .file_name = file_path,
      .function_name = "test_function",
      .log_data = test_msg,
      .log_data_size = strlen(test_msg)

  };

  LogFormatter formatter;
  std::string actual_output;
  std::string expected_output;

  formatter.SetPattern("[%c.%f][%l][%t][%n][%g:%R @%F]%v");
#ifdef _WIN32
  expected_output = "[2024-10-01 10:10:10.000000][Info][1234][test_module][XX\\YY\\ZZ\\test_file.cpp:20 @test_function]test log message";
#else
  expected_output = "[2024-10-01 10:10:10.000000][Info][1234][test_module][XX/YY/ZZ/test_file.cpp:20 @test_function]test log message";
#endif
  actual_output = formatter.Format(log_data_wrapper);
  EXPECT_EQ(actual_output, expected_output);
}

// Test for microsecond with part value
TEST(FORMATTER_MICROSECOND_TEST_PART, Format_microsecond_test_part) {
  auto date = std::chrono::year(2024) / std::chrono::month(10) / std::chrono::day(1);
  auto time = std::chrono::hours(10) +
              std::chrono::minutes(10) +
              std::chrono::seconds(10) +
              std::chrono::microseconds(12);

  // // 2024-10-01 10:10:10.000012
  auto datatime = std::chrono::sys_days(date) + time - std::chrono::seconds(common::util::GetLocalTimeZone());

  const char* test_msg = "test log message";
#ifdef _WIN32
  const char* file_path = "XX\\YY\\ZZ\\test_file.cpp";
#else
  const char* file_path = "XX/YY/ZZ/test_file.cpp";
#endif

  LogDataWrapper log_data_wrapper{
      .module_name = "test_module",
      .thread_id = 1234,
      .t = datatime,
      .lvl = AIMRT_LOG_LEVEL_INFO,
      .line = 20,
      .file_name = file_path,
      .function_name = "test_function",
      .log_data = test_msg,
      .log_data_size = strlen(test_msg)

  };

  LogFormatter formatter;
  std::string actual_output;
  std::string expected_output;

  formatter.SetPattern("[%c.%f][%l][%t][%n][%g:%R @%F]%v");
#ifdef _WIN32
  expected_output = "[2024-10-01 10:10:10.000012][Info][1234][test_module][XX\\YY\\ZZ\\test_file.cpp:20 @test_function]test log message";
#else
  expected_output = "[2024-10-01 10:10:10.000012][Info][1234][test_module][XX/YY/ZZ/test_file.cpp:20 @test_function]test log message";
#endif
  actual_output = formatter.Format(log_data_wrapper);
  EXPECT_EQ(actual_output, expected_output);
}

}  // namespace aimrt::runtime::core::logger