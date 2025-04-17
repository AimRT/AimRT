// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "util/stl_tool.h"
#include "util/string_util.h"

namespace aimrt::common::util {

template <class StringType>
void TestTrim() {
  struct TestCase {
    std::string name;

    StringType s;

    StringType want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .s = " testval  ",
          .want_result = "testval"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .s = " ",
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .s = "",
          .want_result = ""});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = Trim(cur_test_case.s);
    EXPECT_EQ(ret, cur_test_case.want_result)
        << "Test " << cur_test_case.name << " failed, index " << ii;
    EXPECT_EQ(ret, cur_test_case.s)
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, Trim_test) {
  TestTrim<std::string>();
  TestTrim<std::string_view>();
}

TEST(STRING_UTIL_TEST, ReplaceString_test) {
  struct TestCase {
    std::string name;

    std::string str;
    std::string_view ov;
    std::string_view nv;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .str = "val1+val2+val3",
          .ov = "val1",
          .nv = "val10",
          .want_result = "val10+val2+val3"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .str = "val1+val2+val3",
          .ov = "val2",
          .nv = "v2",
          .want_result = "val1+v2+val3"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .str = "val1+val2+val3",
          .ov = "val3",
          .nv = "val4",
          .want_result = "val1+val2+val4"});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 1",
          .str = "val1+val2+val3",
          .ov = "kkk",
          .nv = "ddd",
          .want_result = "val1+val2+val3"});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 2",
          .str = "val1+val2+val3",
          .ov = "",
          .nv = "ddd",
          .want_result = "val1+val2+val3"});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 3",
          .str = "val1+val2+val3",
          .ov = "+",
          .nv = "",
          .want_result = "val1val2val3"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = ReplaceString(cur_test_case.str, cur_test_case.ov, cur_test_case.nv);
    EXPECT_STREQ(ret.c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
    EXPECT_STREQ(ret.c_str(), cur_test_case.str.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, IsAlnumStr_test) {
  struct TestCase {
    std::string name;

    std::string_view str;

    bool want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .str = "123456789",
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .str = "123456789abcd",
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .str = "123456789..",
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .str = "",
          .want_result = false});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = IsAlnumStr(cur_test_case.str);
    EXPECT_EQ(ret, cur_test_case.want_result)
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, IsIsDigitStr_test) {
  struct TestCase {
    std::string name;

    std::string_view str;

    bool want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .str = "123456789",
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .str = "123456789abcd",
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .str = "123456789..",
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .str = "",
          .want_result = false});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = IsDigitStr(cur_test_case.str);
    EXPECT_EQ(ret, cur_test_case.want_result)
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

template <class StringType>
void TestSplitToMap() {
  struct TestCase {
    std::string name;

    std::string_view source;
    std::string_view vsep;
    std::string_view msep;
    bool trim;
    bool clear;

    std::map<StringType, StringType> want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .source = "k1=v1&k2=v2&k3=v3&k4=v4",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .clear = false,
          .want_result = {{"k1", "v1"}, {"k2", "v2"}, {"k3", "v3"}, {"k4", "v4"}}});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .source = "k1=v1& k2 =v2&k3= v3 &k4=v4",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .clear = false,
          .want_result = {{"k1", "v1"}, {"k2", "v2"}, {"k3", "v3"}, {"k4", "v4"}}});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .source = "k1=v1& k2 =v2&k3= v3 &k4=v4",
          .vsep = "&",
          .msep = "=",
          .trim = false,
          .clear = false,
          .want_result = {{"k1", "v1"}, {" k2 ", "v2"}, {"k3", " v3 "}, {"k4", "v4"}}});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .source = "k1=v1&k2=v2&k3=v3&&k4=v4",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .clear = false,
          .want_result = {{"k1", "v1"}, {"k2", "v2"}, {"k3", "v3"}, {"k4", "v4"}}});
  test_cases.emplace_back(
      TestCase{
          .name = "case 5",
          .source = "k1=v1&k2=v2&k3=v3&k4=v4&k4=v4x",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .clear = false,
          .want_result = {{"k1", "v1"}, {"k2", "v2"}, {"k3", "v3"}, {"k4", "v4x"}}});
  test_cases.emplace_back(
      TestCase{
          .name = "case 6",
          .source = "k1=v1&k2=v2&k3=v3&=& =v5&k6= &",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .clear = false,
          .want_result = {{"k1", "v1"}, {"k2", "v2"}, {"k3", "v3"}, {"", "v5"}, {"k6", ""}}});
  test_cases.emplace_back(
      TestCase{
          .name = "case 7",
          .source = "k1=v1&k2=v2&k3=v3&=& =v5&k6= &",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .clear = true,
          .want_result = {{"k1", "v1"}, {"k2", "v2"}, {"k3", "v3"}, {"k6", ""}}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 1",
          .source = "k1== v1&k2=v2&k3=v3",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .clear = false,
          .want_result = {{"k1", "= v1"}, {"k2", "v2"}, {"k3", "v3"}}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 2",
          .source = "k1=v1 = v11&k2=v2&k3=v3",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .clear = false,
          .want_result = {{"k1", "v1 = v11"}, {"k2", "v2"}, {"k3", "v3"}}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 3",
          .source = "&k1=v1&&k2=v2&&&k3=v3",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .clear = false,
          .want_result = {{"k1", "v1"}, {"k2", "v2"}, {"k3", "v3"}}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 4",
          .source = "k0&k1=v1&k2=v2&xxx&k3=v3",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .clear = false,
          .want_result = {{"k1", "v1"}, {"k2", "v2"}, {"k3", "v3"}}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 5",
          .source = "",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .clear = false,
          .want_result = {}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 6",
          .source = "k1=v1",
          .vsep = "",
          .msep = "=",
          .trim = true,
          .clear = false,
          .want_result = {}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 7",
          .source = "k1=v1",
          .vsep = "&",
          .msep = "",
          .trim = true,
          .clear = false,
          .want_result = {}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 8",
          .source = "k1=v1",
          .vsep = "=",
          .msep = "=",
          .trim = true,
          .clear = false,
          .want_result = {}});
  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = SplitToMap<StringType>(cur_test_case.source, cur_test_case.vsep,
                                      cur_test_case.msep, cur_test_case.trim,
                                      cur_test_case.clear);
    EXPECT_TRUE(CheckMapEqual(ret, cur_test_case.want_result))
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, SplitToMap_test) {
  TestSplitToMap<std::string>();
  TestSplitToMap<std::string_view>();
}

template <class StringType>
void TestJoinMap() {
  struct TestCase {
    std::string name;

    std::map<StringType, StringType> m;
    std::string_view vsep;
    std::string_view msep;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .m = {{"k1", "v1"}, {"k2", "v2"}, {"k3", "v3"}},
          .vsep = "&",
          .msep = "=",
          .want_result = "k1=v1&k2=v2&k3=v3"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .m = {{"k1", "v1"}, {"k2", ""}, {"", "v3"}},
          .vsep = "&",
          .msep = "=",
          .want_result = "=v3&k1=v1&k2="});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .m = {{"k1", "v1"}},
          .vsep = "&",
          .msep = "=",
          .want_result = "k1=v1"});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 1",
          .m = {{"", ""}},
          .vsep = "&",
          .msep = "=",
          .want_result = "="});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 2",
          .m = {{"", ""}, {"k1", "v1"}},
          .vsep = "&",
          .msep = "=",
          .want_result = "=&k1=v1"});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 3",
          .m = {{"", ""}, {"k1", "v1"}},
          .vsep = "&",
          .msep = "",
          .want_result = "&k1v1"});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 4",
          .m = {},
          .vsep = "&",
          .msep = "=",
          .want_result = ""});
  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = JoinMap(cur_test_case.m, cur_test_case.vsep, cur_test_case.msep);
    EXPECT_STREQ(ret.c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, JoinMap_test) {
  TestJoinMap<std::string>();
  TestJoinMap<std::string_view>();
}

TEST(STRING_UTIL_TEST, GetValueFromStrKV_test) {
  struct TestCase {
    std::string name;

    std::string_view str;
    std::string_view key;
    std::string_view vsep;
    std::string_view msep;
    bool trim;

    std::string_view want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .str = "k1=v1&k2=v2&k3=v3&k4=v4",
          .key = "k1",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .want_result = "v1"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .str = "k1=v1&k2=v2& k3 = v3 &k4=v4",
          .key = "k3",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .want_result = "v3"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .str = "k1=v1&k2=v2& k3 = v3 &k4=v4",
          .key = "k3",
          .vsep = "&",
          .msep = "=",
          .trim = false,
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .str = "k1=v1&k2=v2&k3=v3&k4= v4 ",
          .key = "k4",
          .vsep = "&",
          .msep = "=",
          .trim = false,
          .want_result = " v4 "});
  test_cases.emplace_back(
      TestCase{
          .name = "case 5",
          .str = "k1=v1&k2=v2&k3=v3&k4=v4",
          .key = "k5",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "case 6",
          .str = "k1=v1&k2=v2&k2=v3&k2=v4",
          .key = "k2",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .want_result = "v2"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 7",
          .str = "k1=v1&kk2=v2&k2 =v3&k2=v4",
          .key = "k2",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .want_result = "v3"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 8",
          .str = "k1=v1&kk2=v2&k2 =v3&k2=v4",
          .key = "k2",
          .vsep = "&",
          .msep = "=",
          .trim = false,
          .want_result = "v4"});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 1",
          .str = "=v1",
          .key = "",
          .vsep = "&",
          .msep = "=",
          .trim = true,
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 2",
          .str = "k1=v1",
          .key = "k1",
          .vsep = "",
          .msep = "=",
          .trim = true,
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 3",
          .str = "k1=v1",
          .key = "k1",
          .vsep = "&",
          .msep = "",
          .trim = true,
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 4",
          .str = "k1=v1",
          .key = "k1",
          .vsep = "=",
          .msep = "=",
          .trim = true,
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 5",
          .str = "k1=v1",
          .key = "k1",
          .vsep = "k",
          .msep = "1",
          .trim = true,
          .want_result = ""});
  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = GetValueFromStrKV(cur_test_case.str, cur_test_case.key,
                                 cur_test_case.vsep, cur_test_case.msep,
                                 cur_test_case.trim);
    EXPECT_EQ(ret, cur_test_case.want_result)
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, GetMapKeys_test) {
  struct TestCase {
    std::string name;

    std::map<std::string, std::string> m;

    std::set<std::string> want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .m = {{"k1", "v1"}, {"k2", "v2"}, {"k3", "v3"}, {"k4", "v4"}},
          .want_result = {"k1", "k2", "k3", "k4"}});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .m = {{"k1", "v1"}, {"k2", "v2"}, {"k3", "v3"}, {"", "v4"}},
          .want_result = {"k1", "k2", "k3", ""}});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .m = {},
          .want_result = {}});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = GetMapKeys(cur_test_case.m);
    EXPECT_TRUE(CheckContainerEqual(ret, cur_test_case.want_result))
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

template <class StringType>
void TestDrawTable() {
  struct TestCase {
    std::string name;

    std::vector<std::vector<StringType>> table;
    bool with_header;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .table = {},
          .with_header = true,
          .want_result = "\n<empty table>"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .table = {{"111", "222", "333"},
                    {"4444", "5555", "6666"},
                    {"77777", "88888", "99999"}},
          .with_header = true,
          .want_result = R"str(
+-------+-------+-------+
| 111   | 222   | 333   |
+-------+-------+-------+
| 4444  | 5555  | 6666  |
| 77777 | 88888 | 99999 |
+-------+-------+-------+
)str"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .table = {{"111"},
                    {"4444", "5555", "6666"},
                    {"77777", "88888"}},
          .with_header = false,
          .want_result = R"str(
+-------+-------+------+
| 111   |       |      |
| 4444  | 5555  | 6666 |
| 77777 | 88888 |      |
+-------+-------+------+
)str"});

  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .table = {{"111", "222", "333"},
                    {"4444", "5\n55\n5", "6666"},
                    {"77777", "88888", "99999"}},
          .with_header = true,
          .want_result = R"str(
+-------+-------+-------+
| 111   | 222   | 333   |
+-------+-------+-------+
| 4444  | 5     | 6666  |
|       | 55    |       |
|       | 5     |       |
| 77777 | 88888 | 99999 |
+-------+-------+-------+
)str"});

  test_cases.emplace_back(
      TestCase{
          .name = "case 5",
          .table = {{"111", "22222", "333"},
                    {"4444", "5\n55\n5", "666\n6"},
                    {"77777", "888\n88", "99999\n9"}},
          .with_header = true,
          .want_result = R"str(
+-------+-------+-------+
| 111   | 22222 | 333   |
+-------+-------+-------+
| 4444  | 5     | 666   |
|       | 55    | 6     |
|       | 5     |       |
| 77777 | 888   | 99999 |
|       | 88    | 9     |
+-------+-------+-------+
)str"});

  test_cases.emplace_back(
      TestCase{
          .name = "case 6",
          .table = {{"111", "22\n2", "333"},
                    {"4444", "5\n55\n5", "666\n6"},
                    {"77777", "88888", "99999"}},
          .with_header = true,
          .want_result = R"str(
+-------+-------+-------+
| 111   | 22    | 333   |
|       | 2     |       |
+-------+-------+-------+
| 4444  | 5     | 666   |
|       | 55    | 6     |
|       | 5     |       |
| 77777 | 88888 | 99999 |
+-------+-------+-------+
)str"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = "\n" + DrawTable<StringType>(cur_test_case.table, cur_test_case.with_header);
    EXPECT_EQ(ret, cur_test_case.want_result)
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, DrawTable_test) {
  TestDrawTable<std::string>();
  TestDrawTable<std::string_view>();
}

template <class StringType>
void TestSplitToVec() {
  struct TestCase {
    std::string name;

    std::string_view source;
    std::string_view sep;
    bool trim;
    bool clear;

    std::vector<StringType> want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .source = "v1, , v3,v4,,v6",
          .sep = ",",
          .trim = true,
          .clear = false,
          .want_result = {"v1", "", "v3", "v4", "", "v6"}});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .source = "v1, , v3,v4,,v6",
          .sep = ",",
          .trim = false,
          .clear = false,
          .want_result = {"v1", " ", " v3", "v4", "", "v6"}});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .source = "v1, , v3,v4,,v6",
          .sep = ",",
          .trim = true,
          .clear = true,
          .want_result = {"v1", "v3", "v4", "v6"}});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .source = "v1, , v3,v4,,v6",
          .sep = ",",
          .trim = false,
          .clear = true,
          .want_result = {"v1", " ", " v3", "v4", "v6"}});

  test_cases.emplace_back(
      TestCase{
          .name = "bad case 1",
          .source = "",
          .sep = ",",
          .trim = false,
          .clear = false,
          .want_result = {}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 2",
          .source = " ",
          .sep = ",",
          .trim = false,
          .clear = false,
          .want_result = {" "}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 3",
          .source = " ",
          .sep = ",",
          .trim = true,
          .clear = false,
          .want_result = {""}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 4",
          .source = " ",
          .sep = ",",
          .trim = true,
          .clear = true,
          .want_result = {}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 5",
          .source = "k1,k2",
          .sep = "",
          .trim = false,
          .clear = false,
          .want_result = {}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 6",
          .source = ",",
          .sep = ",",
          .trim = false,
          .clear = false,
          .want_result = {"", ""}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 7",
          .source = " k1 k2  k3 ",
          .sep = " ",
          .trim = false,
          .clear = false,
          .want_result = {"", "k1", "k2", "", "k3", ""}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 8",
          .source = " k1 k2  k3 ",
          .sep = " ",
          .trim = true,
          .clear = false,
          .want_result = {"", "k1", "k2", "", "k3", ""}});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = SplitToVec<StringType>(cur_test_case.source, cur_test_case.sep,
                                      cur_test_case.trim, cur_test_case.clear);
    EXPECT_TRUE(CheckContainerEqual(ret, cur_test_case.want_result))
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, SplitToVec_test) {
  TestSplitToVec<std::string>();
  TestSplitToVec<std::string_view>();
}

template <class StringType>
void TestJoinVec() {
  struct TestCase {
    std::string name;

    std::vector<StringType> vec;
    std::string_view sep;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .vec = {"v1", "v2", "v3", "v4"},
          .sep = "|",
          .want_result = "v1|v2|v3|v4"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .vec = {"v1", "v2", "v3", "v4"},
          .sep = "",
          .want_result = "v1v2v3v4"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .vec = {"", "", "", "v4"},
          .sep = "",
          .want_result = "v4"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .vec = {"", "", "", ""},
          .sep = "",
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "case 5",
          .vec = {"", "", "", "v4"},
          .sep = ",",
          .want_result = ",,,v4"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 6",
          .vec = {"", "v2", "", "v4"},
          .sep = ",",
          .want_result = ",v2,,v4"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = JoinVec(cur_test_case.vec, cur_test_case.sep);
    EXPECT_STREQ(ret.c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, JoinVec_test) {
  TestJoinVec<std::string>();
  TestJoinVec<std::string_view>();
}

TEST(STRING_UTIL_TEST, CheckIfInList_test) {
  struct TestCase {
    std::string name;

    std::string_view str;
    std::string_view key;
    std::string_view sep;
    bool trim;

    bool want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .str = " 123456 ",
          .key = "123456",
          .sep = ",",
          .trim = true,
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .str = "0123456",
          .key = "123456",
          .sep = ",",
          .trim = true,
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .str = "1234567",
          .key = "123456",
          .sep = ",",
          .trim = true,
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .str = " 123456 , ",
          .key = "123456",
          .sep = ",",
          .trim = true,
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 5",
          .str = " , 123456 ",
          .key = "123456",
          .sep = ",",
          .trim = true,
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 6",
          .str = "aaa, 123456, bbb",
          .key = "123456",
          .sep = ",",
          .trim = true,
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 7",
          .str = "aaa, 123456,bbb",
          .key = "123456",
          .sep = ",",
          .trim = false,
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 8",
          .str = "aaa,123456 ,bbb",
          .key = "123456",
          .sep = ",",
          .trim = false,
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 9",
          .str = "aaa, 1234567 , 123456 ,bbb",
          .key = "123456",
          .sep = ",",
          .trim = true,
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 10",
          .str = "aaa, 123456,bbb",
          .key = " 123456",
          .sep = ",",
          .trim = true,
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 11",
          .str = "aaa, 123456,bbb",
          .key = " 123456",
          .sep = ",",
          .trim = false,
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 1",
          .str = "123,456",
          .key = "123,456",
          .sep = ",",
          .trim = true,
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 2",
          .str = "aaa,,bbb",
          .key = "",
          .sep = ",",
          .trim = true,
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 3",
          .str = "aaa,bbb",
          .key = "aaa",
          .sep = "",
          .trim = false,
          .want_result = false});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = CheckIfInList(cur_test_case.str, cur_test_case.key,
                             cur_test_case.sep, cur_test_case.trim);
    EXPECT_EQ(ret, cur_test_case.want_result)
        << "Test " << cur_test_case.name << " failed, index " << ii;
    ;
  }
}

template <class StringType>
void TestSplitToSet() {
  struct TestCase {
    std::string name;

    std::string_view source;
    std::string_view sep;
    bool trim;
    bool clear;

    std::set<StringType> want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .source = "v1, , v3,v4,,v6",
          .sep = ",",
          .trim = true,
          .clear = false,
          .want_result = {"v1", "", "v3", "v4", "v6"}});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .source = "v1, , v3,v4,,v6",
          .sep = ",",
          .trim = false,
          .clear = false,
          .want_result = {"v1", " ", " v3", "v4", "", "v6"}});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .source = "v1, , v3,v4,,v6",
          .sep = ",",
          .trim = true,
          .clear = true,
          .want_result = {"v1", "v3", "v4", "v6"}});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .source = "v1, , v3,v4,,v6",
          .sep = ",",
          .trim = false,
          .clear = true,
          .want_result = {"v1", " ", " v3", "v4", "v6"}});

  test_cases.emplace_back(
      TestCase{
          .name = "bad case 1",
          .source = "",
          .sep = ",",
          .trim = false,
          .clear = false,
          .want_result = {}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 2",
          .source = " ",
          .sep = ",",
          .trim = false,
          .clear = false,
          .want_result = {" "}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 3",
          .source = " ",
          .sep = ",",
          .trim = true,
          .clear = false,
          .want_result = {""}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 4",
          .source = " ",
          .sep = ",",
          .trim = true,
          .clear = true,
          .want_result = {}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 5",
          .source = "k1,k2",
          .sep = "",
          .trim = false,
          .clear = false,
          .want_result = {}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 6",
          .source = ",",
          .sep = ",",
          .trim = false,
          .clear = false,
          .want_result = {""}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 7",
          .source = " k1 k2  k3 ",
          .sep = " ",
          .trim = false,
          .clear = false,
          .want_result = {"", "k1", "k2", "k3"}});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 8",
          .source = " k1 k2  k3 ",
          .sep = " ",
          .trim = true,
          .clear = false,
          .want_result = {"", "k1", "k2", "k3"}});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = SplitToSet<StringType>(cur_test_case.source, cur_test_case.sep,
                                      cur_test_case.trim, cur_test_case.clear);
    EXPECT_TRUE(CheckContainerEqual(ret, cur_test_case.want_result))
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, SplitToSet_test) {
  TestSplitToSet<std::string>();
  TestSplitToSet<std::string_view>();
}

template <class StringType>
void TestJoinSet() {
  struct TestCase {
    std::string name;

    std::set<StringType> st;
    std::string_view sep;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .st = {"v1", "v2", "v3", "v4"},
          .sep = "|",
          .want_result = "v1|v2|v3|v4"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .st = {"v1", "v2", "v3", "v4"},
          .sep = "",
          .want_result = "v1v2v3v4"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .st = {"", "v4"},
          .sep = "",
          .want_result = "v4"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .st = {""},
          .sep = "",
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "case 5",
          .st = {"", "v4"},
          .sep = ",",
          .want_result = ",v4"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 6",
          .st = {"", "v2", "v4"},
          .sep = ",",
          .want_result = ",v2,v4"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = JoinSet(cur_test_case.st, cur_test_case.sep);
    EXPECT_STREQ(ret.c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, JoinSet_test) {
  TestJoinSet<std::string>();
  TestJoinSet<std::string_view>();
}

TEST(STRING_UTIL_TEST, CmpVersion_test) {
  struct TestCase {
    std::string name;

    std::string_view ver1;
    std::string_view ver2;

    int want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .ver1 = "7.6.8.11020",
          .ver2 = "7.6.8",
          .want_result = 1});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .ver1 = "7.6.8",
          .ver2 = "7.6.8",
          .want_result = 0});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .ver1 = "7.6.8",
          .ver2 = "7.6.8.11020",
          .want_result = -1});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 1",
          .ver1 = "9.8.0",
          .ver2 = "9..8.0",
          .want_result = 0});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 2",
          .ver1 = "",
          .ver2 = "0.0.0",
          .want_result = -1});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = CmpVersion(cur_test_case.ver1, cur_test_case.ver2);
    EXPECT_EQ(ret, cur_test_case.want_result)
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, CheckVersionInside_test) {
  struct TestCase {
    std::string name;

    std::string_view ver;
    std::string_view start_ver;
    std::string_view end_ver;

    bool want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .ver = "7.6.8",
          .start_ver = "7.6.7",
          .end_ver = "7.6.9",
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .ver = "7.6.8",
          .start_ver = "7.6.8",
          .end_ver = "7.6.8",
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .ver = "7.6.5",
          .start_ver = "7.6.7",
          .end_ver = "7.6.9",
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 1",
          .ver = "7.6.8",
          .start_ver = "",
          .end_ver = "",
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 2",
          .ver = "0.0.0.0",
          .start_ver = "",
          .end_ver = "",
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "bad case 3",
          .ver = "999.9.9.9",
          .start_ver = "",
          .end_ver = "",
          .want_result = true});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = CheckVersionInside(cur_test_case.ver, cur_test_case.start_ver,
                                  cur_test_case.end_ver);
    EXPECT_EQ(ret, cur_test_case.want_result)
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

template <class StringType>
void TestGetMapItemWithDef() {
  struct TestCase {
    std::string name;

    std::map<StringType, StringType> m;
    StringType key;
    StringType defval;

    StringType want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .m = {{"k1", "v1"}, {"k2", "v2"}},
          .key = "k1",
          .defval = "",
          .want_result = "v1"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .m = {{"k1", "v1"}, {"k2", "v2"}},
          .key = "k3",
          .defval = "",
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .m = {{"k1", "v1"}, {"k2", "v2"}},
          .key = "",
          .defval = "abc",
          .want_result = "abc"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = GetMapItemWithDef(cur_test_case.m, cur_test_case.key,
                                 cur_test_case.defval);
    EXPECT_EQ(ret, cur_test_case.want_result)
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, GetMapItemWithDef_test) {
  TestGetMapItemWithDef<std::string>();
  TestGetMapItemWithDef<std::string_view>();
}

TEST(STRING_UTIL_TEST, AddKV_test) {
  struct TestCase {
    std::string name;

    std::string s;
    std::string_view key;
    std::string_view val;
    std::string_view vsep;
    std::string_view msep;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;

  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .s = "",
          .key = "k1",
          .val = "v1",
          .vsep = "&",
          .msep = "=",
          .want_result = "k1=v1"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .s = "aaa",
          .key = "k1",
          .val = "v1",
          .vsep = "&",
          .msep = "=",
          .want_result = "aaa&k1=v1"});
  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = AddKV(cur_test_case.s, cur_test_case.key, cur_test_case.val,
                     cur_test_case.vsep, cur_test_case.msep);
    EXPECT_STREQ(ret.c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, StrToLower_test) {
  struct TestCase {
    std::string name;

    std::string str;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .str = "ABCDEFG",
          .want_result = "abcdefg"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .str = "123456 ",
          .want_result = "123456 "});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = StrToLower(cur_test_case.str);
    EXPECT_STREQ(ret.c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, StrToUpper_test) {
  struct TestCase {
    std::string name;

    std::string str;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .str = "abcdefg",
          .want_result = "ABCDEFG"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .str = "123456 ",
          .want_result = "123456 "});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = StrToUpper(cur_test_case.str);
    EXPECT_STREQ(ret.c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, CheckIEqual_test) {
  struct TestCase {
    std::string name;

    std::string str1;
    std::string str2;

    bool want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .str1 = "",
          .str2 = "",
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .str1 = "a",
          .str2 = "",
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .str1 = "abc",
          .str2 = "AbC",
          .want_result = true});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = CheckIEqual(cur_test_case.str1, cur_test_case.str2);
    EXPECT_EQ(ret, cur_test_case.want_result)
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, StrToTitleCase_test) {
  struct TestCase {
    std::string name;

    std::string str;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .str = "abc defg",
          .want_result = "Abc Defg"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .str = "123 456",
          .want_result = "123 456"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = StrToTitleCase(cur_test_case.str);
    EXPECT_STREQ(ret.c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, StandardisePath_test) {
  struct TestCase {
    std::string name;

    std::string path;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .path = "",
          .want_result = "/"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .path = "root",
          .want_result = "root/"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .path = "root/",
          .want_result = "root/"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .path = "C\\work",
          .want_result = "C/work/"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = StandardisePath(cur_test_case.path);
    EXPECT_STREQ(ret.c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, StartsWith_test) {
  struct TestCase {
    std::string name;

    std::string_view str;
    std::string_view pattern;
    bool ignore_case;

    bool want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .str = "abcdefg123456",
          .pattern = "abc",
          .ignore_case = false,
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .str = "abcdefg123456",
          .pattern = "abC",
          .ignore_case = false,
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .str = "abcdefg123456",
          .pattern = "abC",
          .ignore_case = true,
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .str = "abcdefg123456",
          .pattern = "",
          .ignore_case = true,
          .want_result = false});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = StartsWith(cur_test_case.str, cur_test_case.pattern,
                          cur_test_case.ignore_case);
    EXPECT_EQ(ret, cur_test_case.want_result)
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, EndsWith_test) {
  struct TestCase {
    std::string name;

    std::string_view str;
    std::string_view pattern;
    bool ignore_case;

    bool want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .str = "123456abcdefg",
          .pattern = "efg",
          .ignore_case = false,
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .str = "123456abcdefg",
          .pattern = "Efg",
          .ignore_case = false,
          .want_result = false});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .str = "123456abcdefg",
          .pattern = "Efg",
          .ignore_case = true,
          .want_result = true});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .str = "123456abcdefg",
          .pattern = "",
          .ignore_case = true,
          .want_result = false});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = EndsWith(cur_test_case.str, cur_test_case.pattern,
                        cur_test_case.ignore_case);
    EXPECT_EQ(ret, cur_test_case.want_result)
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, Hash64Fnv1a_test) {
  struct TestCase {
    std::string name;

    std::string_view data;

    uint64_t want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .data = "abcdefg123456",
          .want_result = 340744521186200064});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .data = "",
          .want_result = 14695981039346656037});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret =
        Hash64Fnv1a(cur_test_case.data.data(), cur_test_case.data.size());
    EXPECT_EQ(ret, cur_test_case.want_result)
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(STRING_UTIL_TEST, Hash32Fnv1a_test) {
  struct TestCase {
    std::string name;

    std::string_view data;

    uint32_t want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .data = "abcdefg123456",
          .want_result = 1667236096});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .data = "",
          .want_result = 2166136261});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret =
        Hash32Fnv1a(cur_test_case.data.data(), cur_test_case.data.size());
    EXPECT_EQ(ret, cur_test_case.want_result)
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

TEST(UTF8_TRUNCATION_TEST, Utf8_Truncation_Test) {
  // Empty string tests
  {
    const char* empty_str = "";
    EXPECT_EQ(SafeUtf8TruncationLength(empty_str, 0, 0), 0);
    EXPECT_EQ(SafeUtf8TruncationLength(empty_str, 0, 5), 0);
  }

  // Pure ASCII tests (single-byte)
  {
    const char* ascii_str = "Hello";
    EXPECT_EQ(SafeUtf8TruncationLength(ascii_str, 5, 3), 3);  // Normal truncation
    EXPECT_EQ(SafeUtf8TruncationLength(ascii_str, 5, 5), 5);  // Exact length
    EXPECT_EQ(SafeUtf8TruncationLength(ascii_str, 5, 6), 5);  // Exceed length
  }

  // Chinese characters test (3-byte)
  {
    // "你好" UTF-8: E4 BD A0 E5 A5 BD
    const char* chinese_str = "\xE4\xBD\xA0\xE5\xA5\xBD";
    const size_t full_len = 6;

    // Truncate in middle of first character
    EXPECT_EQ(SafeUtf8TruncationLength(chinese_str, full_len, 1), 0);
    EXPECT_EQ(SafeUtf8TruncationLength(chinese_str, full_len, 2), 0);

    // Truncate at character boundary
    EXPECT_EQ(SafeUtf8TruncationLength(chinese_str, full_len, 3), 3);  // Complete "你"

    // Truncate in middle of second character
    EXPECT_EQ(SafeUtf8TruncationLength(chinese_str, full_len, 4), 3);
    EXPECT_EQ(SafeUtf8TruncationLength(chinese_str, full_len, 5), 3);
  }

  // 4-byte character test (emoji)
  {
    // U+1F600 (😀) UTF-8: F0 9F 98 80
    const char* emoji_str = "\xF0\x9F\x98\x80";
    const size_t full_len = 4;

    // Progressive truncation
    EXPECT_EQ(SafeUtf8TruncationLength(emoji_str, full_len, 1), 0);
    EXPECT_EQ(SafeUtf8TruncationLength(emoji_str, full_len, 2), 0);
    EXPECT_EQ(SafeUtf8TruncationLength(emoji_str, full_len, 3), 0);
    EXPECT_EQ(SafeUtf8TruncationLength(emoji_str, full_len, 4), 4);  // Full retention
  }

  // Mixed-length characters test
  {
    // "A你好" = A (1) + 你 (3) + 好 (3)
    const char* mixed_str = "A\xE4\xBD\xA0\xE5\xA5\xBD";
    const size_t full_len = 7;

    // Truncate after ASCII
    EXPECT_EQ(SafeUtf8TruncationLength(mixed_str, full_len, 1), 1);  // "A"

    // Truncate in middle of Chinese
    EXPECT_EQ(SafeUtf8TruncationLength(mixed_str, full_len, 2), 1);
    EXPECT_EQ(SafeUtf8TruncationLength(mixed_str, full_len, 4), 4);  // "A你"

    // Truncate at last character
    EXPECT_EQ(SafeUtf8TruncationLength(mixed_str, full_len, 7), 7);
  }

  // Edge case testing
  {
    // Invalid UTF-8 sequence (handled correctly)
    const char* invalid_utf8 = "\xE4\xBD";  // Incomplete Chinese character
    EXPECT_EQ(SafeUtf8TruncationLength(invalid_utf8, 2, 1), 0);
    EXPECT_EQ(SafeUtf8TruncationLength(invalid_utf8, 2, 2), 2);
  }

  // Extreme value testing
  {
    const char* max_str = "test";
    EXPECT_EQ(SafeUtf8TruncationLength(max_str, 4, SIZE_MAX), 4);
  }
}

}  // namespace aimrt::common::util
