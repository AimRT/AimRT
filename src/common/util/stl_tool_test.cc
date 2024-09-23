// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "util/stl_tool.h"

namespace aimrt::common::util {

TEST(STL_TOOL_TEST, Vec2Str_test) {
  struct TestCase {
    std::string name;

    std::vector<std::string> v;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .v = {},
          .want_result = R"str(size = 0
)str"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .v = {"", "v1", "v2\nv2", "12345678901234567890123456789012",
                "123456789012345678901234567890123", "v3"},
          .want_result = R"str(size = 6
[index=0]:<empty string>
[index=1]:v1
[index=2]:
v2
v2
[index=3]:12345678901234567890123456789012
[index=4]:
123456789012345678901234567890123
[index=5]:v3
)str"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    auto ret = Container2Str(test_cases[ii].v);
    EXPECT_STREQ(ret.c_str(), test_cases[ii].want_result.c_str())
        << "Test " << test_cases[ii].name << " failed, index " << ii;
  }
}

TEST(STL_TOOL_TEST, Vec2Str_bool_test) {
  struct TestCase {
    std::string name;

    std::vector<bool> v;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .v = {},
          .want_result = R"str(size = 0
)str"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .v = {true, true, false, false, true, true},
          .want_result = R"str(size = 6
[index=0]:1
[index=1]:1
[index=2]:0
[index=3]:0
[index=4]:1
[index=5]:1
)str"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    auto ret = Container2Str(test_cases[ii].v);
    EXPECT_STREQ(ret.c_str(), test_cases[ii].want_result.c_str())
        << "Test " << test_cases[ii].name << " failed, index " << ii;
  }
}

TEST(STL_TOOL_TEST, Set2Str_test) {
  struct TestCase {
    std::string name;

    std::set<std::string> s;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .s = {},
          .want_result = R"str(size = 0
)str"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .s = {"", "v1", "v2\nv2", "12345678901234567890123456789012",
                "123456789012345678901234567890123", "v3"},
          .want_result = R"str(size = 6
[index=0]:<empty string>
[index=1]:12345678901234567890123456789012
[index=2]:
123456789012345678901234567890123
[index=3]:v1
[index=4]:
v2
v2
[index=5]:v3
)str"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    auto ret = Container2Str(test_cases[ii].s);
    EXPECT_STREQ(ret.c_str(), test_cases[ii].want_result.c_str())
        << "Test " << test_cases[ii].name << " failed, index " << ii;
  }
}

TEST(STL_TOOL_TEST, Map2Str_test) {
  struct TestCase {
    std::string name;

    std::map<std::string, std::string> m;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .m = {},
          .want_result = R"str(size = 0
)str"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .m = {{"k1", "v1"},
                {"k2", ""},
                {"", "v3"},
                {"k4\nk4", "v4"},
                {"k5", "v5\nv5"}},
          .want_result = R"str(size = 5
[index=0]:
  [key]:<empty string>
  [val]:v3
[index=1]:
  [key]:k1
  [val]:v1
[index=2]:
  [key]:k2
  [val]:<empty string>
[index=3]:
  [key]:
k4
k4
  [val]:v4
[index=4]:
  [key]:k5
  [val]:
v5
v5
)str"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .m = {{"123456789012345678901234567890123", "v1"},
                {"k2", "123456789012345678901234567890123"}},
          .want_result = R"str(size = 2
[index=0]:
  [key]:
123456789012345678901234567890123
  [val]:v1
[index=1]:
  [key]:k2
  [val]:
123456789012345678901234567890123
)str"});
  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    auto ret = Map2Str(test_cases[ii].m);
    EXPECT_STREQ(ret.c_str(), test_cases[ii].want_result.c_str())
        << "Test " << test_cases[ii].name << " failed, index " << ii;
  }
}

TEST(STL_TOOL_TEST, CheckVectorEqual_test) {
  struct TestCase {
    std::string name;

    std::vector<std::string> vec1;
    std::vector<std::string> vec2;

    bool want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .vec1 = {"v1", "v2"},
          .vec2 = {"v1", "v2"},
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .vec1 = {"v1", "v2"},
          .vec2 = {"v1", "v2", "v3"},
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .vec1 = {"v1", "v2"},
          .vec2 = {"v1", "v2x"},
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .vec1 = {},
          .vec2 = {},
          .want_result = true});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    auto ret = CheckContainerEqual(test_cases[ii].vec1, test_cases[ii].vec2);
    EXPECT_EQ(ret, test_cases[ii].want_result)
        << "Test " << test_cases[ii].name << " failed, index " << ii;
  }
}

TEST(STL_TOOL_TEST, CheckVectorEqual_bool_test) {
  struct TestCase {
    std::string name;

    std::vector<bool> vec1;
    std::vector<bool> vec2;

    bool want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .vec1 = {true, false},
          .vec2 = {true, false},
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .vec1 = {true, false},
          .vec2 = {true, false, true},
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .vec1 = {true, false},
          .vec2 = {true, true},
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .vec1 = {},
          .vec2 = {},
          .want_result = true});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    auto ret = CheckContainerEqual(test_cases[ii].vec1, test_cases[ii].vec2);
    EXPECT_EQ(ret, test_cases[ii].want_result)
        << "Test " << test_cases[ii].name << " failed, index " << ii;
  }
}

TEST(STL_TOOL_TEST, CheckSetEqual_test) {
  struct TestCase {
    std::string name;

    std::set<std::string> set1;
    std::set<std::string> set2;

    bool want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .set1 = {"v1", "v2"},
          .set2 = {"v1", "v2"},
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .set1 = {"v1", "v2"},
          .set2 = {"v1", "v2", "v3"},
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .set1 = {"v1", "v2"},
          .set2 = {"v1", "v2x"},
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .set1 = {},
          .set2 = {},
          .want_result = true});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    auto ret = CheckContainerEqual(test_cases[ii].set1, test_cases[ii].set2);
    EXPECT_EQ(ret, test_cases[ii].want_result)
        << "Test " << test_cases[ii].name << " failed, index " << ii;
  }
}

TEST(STL_TOOL_TEST, CheckMapEqual_test) {
  struct TestCase {
    std::string name;

    std::map<std::string, std::string> map1;
    std::map<std::string, std::string> map2;

    bool want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .map1 = {},
          .map2 = {},
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .map1 = {{"k1", "v1"}, {"k2", "v2"}},
          .map2 = {{"k1", "v1"}, {"k2", "v2"}},
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .map1 = {{"k1", "v1"}, {"k2", "v2"}},
          .map2 = {{"k1", "v1"}, {"k2", "v2x"}},
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .map1 = {{"k1", "v1"}, {"k2", "v2"}},
          .map2 = {{"k1", "v1"}, {"k2", "v2"}, {"k3", "v3"}},
          .want_result = false});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    auto ret = CheckMapEqual(test_cases[ii].map1, test_cases[ii].map2);
    EXPECT_EQ(ret, test_cases[ii].want_result)
        << "Test " << test_cases[ii].name << " failed, index " << ii;
  }
}

}  // namespace aimrt::common::util
