// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "util/url_encode.h"

namespace aimrt::common::util {

TEST(URL_ENCODE_TEST, UrlEncode_test) {
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
          .str = "http://abc123.com/aaa/bbbb?qa=1&qb=adf",
          .up = true,
          .want_result = "http%3A%2F%2Fabc123.com%2Faaa%2Fbbbb%3Fqa%3D1%26qb%3Dadf"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .str = "http://abc123.com/aaa/bbbb?qa=1&qb=adf",
          .up = false,
          .want_result = "http%3a%2f%2fabc123.com%2faaa%2fbbbb%3fqa%3d1%26qb%3dadf"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = UrlEncode(cur_test_case.str, cur_test_case.up);
    EXPECT_STREQ(ret.c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(URL_ENCODE_TEST, UrlDecode_test) {
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
          .str = "http%3A%2F%2Fabc123.com%2Faaa%2Fbbbb%3Fqa%3D1%26qb%3Dadf",
          .want_result = "http://abc123.com/aaa/bbbb?qa=1&qb=adf"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .str = "http%3a%2f%2fabc123.com%2faaa%2fbbbb%3fqa%3d1%26qb%3dadf",
          .want_result = "http://abc123.com/aaa/bbbb?qa=1&qb=adf"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = UrlDecode(cur_test_case.str);
    EXPECT_STREQ(ret.c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

}  // namespace aimrt::common::util