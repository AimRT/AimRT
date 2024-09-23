// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "ros2_plugin/ros2_name_encode.h"

namespace aimrt::plugins::ros2_plugin {

TEST(ROS2_ENCODE_TEST, Ros2NameEncode_test) {
  struct TestCase {
    std::string name;

    std::string str;
    bool up;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .str = "",
          .up = true,
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .str = "aaa.bbb.ccc/ddd",
          .up = true,
          .want_result = "aaa_2Ebbb_2Eccc/ddd"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = Ros2NameEncode(cur_test_case.str, cur_test_case.up);
    EXPECT_STREQ(ret.c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(ROS2_ENCODE_TEST, Ros2NameDecode_test) {
  struct TestCase {
    std::string name;

    std::string str;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .str = "",
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .str = "aaa_2Ebbb_2Eccc/ddd",
          .want_result = "aaa.bbb.ccc/ddd"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = Ros2NameDecode(cur_test_case.str);
    EXPECT_STREQ(ret.c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

}  // namespace aimrt::plugins::ros2_plugin