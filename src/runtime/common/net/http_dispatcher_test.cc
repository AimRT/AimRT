// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "net/http_dispatcher.h"

namespace aimrt::runtime::common::net {

TEST(HTTP_DISPATCHER_TEST, HttpDispatcher_CASE1) {
  using TestHttpDispatcher = HttpDispatcher<std::string(void)>;

  TestHttpDispatcher dispatcher;
  dispatcher.RegisterHttpHandle(
      "",
      []() -> std::string { return "CASE 1"; });

  dispatcher.RegisterHttpHandle(
      "/",
      []() -> std::string { return "CASE 2"; });

  dispatcher.RegisterHttpHandle(
      "/ABC",
      []() -> std::string { return "CASE 3"; });

  dispatcher.RegisterHttpHandle(
      "/.*",
      []() -> std::string { return "CASE 4"; });

  struct TestCase {
    std::string name;

    std::string path;

    bool want_match;
    std::string want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .path = "",
          .want_match = true,
          .want_result = "CASE 1"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .path = "/",
          .want_match = true,
          .want_result = "CASE 2"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .path = "/ABC",
          .want_match = true,
          .want_result = "CASE 3"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .path = "/xxxxxx",
          .want_match = true,
          .want_result = "CASE 4"});
  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = dispatcher.GetHttpHandle(cur_test_case.path);
    EXPECT_EQ(static_cast<bool>(ret), cur_test_case.want_match)
        << "Test " << cur_test_case.name << " failed, index " << ii;
    EXPECT_STREQ(ret().c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(HTTP_DISPATCHER_TEST, HttpDispatcher_CASE2) {
  using TestHttpDispatcher = HttpDispatcher<std::string(void)>;

  TestHttpDispatcher dispatcher;
  dispatcher.RegisterHttpHandle(
      "/ABC",
      []() -> std::string { return "CASE 1"; });

  dispatcher.RegisterHttpHandle(
      "/abc",
      []() -> std::string { return "CASE 2"; });

  dispatcher.RegisterHttpHandle(
      "/ABc",
      []() -> std::string { return "CASE 3"; });

  struct TestCase {
    std::string name;

    std::string path;

    bool want_match;
    std::string want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .path = "",
          .want_match = false,
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .path = "/",
          .want_match = false,
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .path = "/ABC",
          .want_match = true,
          .want_result = "CASE 1"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .path = "/abc",
          .want_match = true,
          .want_result = "CASE 1"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 5",
          .path = "/ABc",
          .want_match = true,
          .want_result = "CASE 1"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 6",
          .path = "/abC",
          .want_match = true,
          .want_result = "CASE 1"});
  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = dispatcher.GetHttpHandle(cur_test_case.path);
    EXPECT_EQ(static_cast<bool>(ret), cur_test_case.want_match)
        << "Test " << cur_test_case.name << " failed, index " << ii;
    if (ret) {
      EXPECT_STREQ(ret().c_str(), cur_test_case.want_result.c_str())
          << "Test " << cur_test_case.name << " failed, index " << ii;
    }
  }
}

TEST(HTTP_DISPATCHER_TEST, HttpDispatcher_CASE3) {
  using TestHttpDispatcher = HttpDispatcher<std::string(void)>;

  TestHttpDispatcher dispatcher;
  dispatcher.RegisterHttpHandle(
      "/path1/path2/.*",
      []() -> std::string { return "CASE 1"; });

  dispatcher.RegisterHttpHandle(
      "/path1/.*",
      []() -> std::string { return "CASE 2"; });

  dispatcher.RegisterHttpHandle(
      "/path3/path4/.*",
      []() -> std::string { return "CASE 3"; });

  dispatcher.RegisterHttpHandle(
      "/path5/.*/path6",
      []() -> std::string { return "CASE 4"; });

  struct TestCase {
    std::string name;

    std::string path;

    bool want_match;
    std::string want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .path = "/path",
          .want_match = false,
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .path = "/path2",
          .want_match = false,
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .path = "/path1/path2/path100",
          .want_match = true,
          .want_result = "CASE 1"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .path = "/path1/path",
          .want_match = true,
          .want_result = "CASE 2"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 5",
          .path = "/path3/path4",
          .want_match = false,
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "case 6",
          .path = "/path5/path101/path102/path6",
          .want_match = true,
          .want_result = "CASE 4"});
  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = dispatcher.GetHttpHandle(cur_test_case.path);
    EXPECT_EQ(static_cast<bool>(ret), cur_test_case.want_match)
        << "Test " << cur_test_case.name << " failed, index " << ii;
    if (ret) {
      EXPECT_STREQ(ret().c_str(), cur_test_case.want_result.c_str())
          << "Test " << cur_test_case.name << " failed, index " << ii;
    }
  }
}

}  // namespace aimrt::runtime::common::net
