// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "util/url_parser.h"

namespace aimrt::common::util {

template <class StringType>
void TestJoinUrl() {
  struct TestCase {
    std::string name;

    Url<StringType> url;

    std::string want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .url = Url<StringType>{},
          .want_result = ""});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .url = Url<StringType>{
              .protocol = "http",
          },
          .want_result = "http://"});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .url = Url<StringType>{
              .protocol = "http",
              .host = "127.0.0.1",
              .service = "80",
              .path = "index.html",
              .query = "key1=val1&key2=val2",
              .fragment = "all",
          },
          .want_result = "http://127.0.0.1:80/index.html?key1=val1&key2=val2#all"});

  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .url = Url<StringType>{
              .path = "index.html",
              .query = "key1=val1&key2=val2",
              .fragment = "all",
          },
          .want_result = "/index.html?key1=val1&key2=val2#all"});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = JoinUrl(cur_test_case.url);
    EXPECT_STREQ(ret.c_str(), cur_test_case.want_result.c_str())
        << "Test " << cur_test_case.name << " failed, index " << ii;
  }
}

template <class StringType>
void TestParseUrl() {
  struct TestCase {
    std::string name;

    std::string url_str;

    std::optional<Url<StringType>> want_result;
  };
  std::vector<TestCase> test_cases;
  test_cases.emplace_back(
      TestCase{
          .name = "case 1",
          .url_str = "",
          .want_result = Url<StringType>{
              .path = "",
          }});
  test_cases.emplace_back(
      TestCase{
          .name = "case 2",
          .url_str = "http://127.0.0.1:80/index.html?key1=val1&key2=val2#all",
          .want_result = Url<StringType>{
              .protocol = "http",
              .host = "127.0.0.1",
              .service = "80",
              .path = "/index.html",
              .query = "key1=val1&key2=val2",
              .fragment = "all",
          }});
  test_cases.emplace_back(
      TestCase{
          .name = "case 3",
          .url_str = "www.abc.com:8080/index.html?key1=val1",
          .want_result = Url<StringType>{
              .host = "www.abc.com",
              .service = "8080",
              .path = "/index.html",
              .query = "key1=val1",
          }});
  test_cases.emplace_back(
      TestCase{
          .name = "case 4",
          .url_str = "www.abc.com:/index.html?key1=val1",
          .want_result = Url<StringType>{
              .host = "www.abc.com",
              .service = "",
              .path = "/index.html",
              .query = "key1=val1",
          }});
  test_cases.emplace_back(
      TestCase{
          .name = "case 5",
          .url_str = "www.abc.com/index.html?key1=val1",
          .want_result = Url<StringType>{
              .host = "www.abc.com",
              .path = "/index.html",
              .query = "key1=val1",
          }});
  test_cases.emplace_back(
      TestCase{
          .name = "case 6",
          .url_str = "www.abc.com?key1=val1",
          .want_result = Url<StringType>{
              .host = "www.abc.com",
              .path = "",
              .query = "key1=val1",
          }});
  test_cases.emplace_back(
      TestCase{
          .name = "case 7",
          .url_str = "xxxxx",
          .want_result = Url<StringType>{
              .host = "xxxxx",
              .path = "",
          }});

  for (size_t ii = 0; ii < test_cases.size(); ++ii) {
    TestCase& cur_test_case = test_cases[ii];
    auto ret = ParseUrl(cur_test_case.url_str);
    EXPECT_EQ(static_cast<bool>(ret),
              static_cast<bool>(cur_test_case.want_result));
    if (ret && cur_test_case.want_result) {
      EXPECT_EQ(ret->protocol, cur_test_case.want_result->protocol)
          << "Test " << cur_test_case.name << " failed, index " << ii;
      EXPECT_EQ(ret->host, cur_test_case.want_result->host)
          << "Test " << cur_test_case.name << " failed, index " << ii;
      EXPECT_EQ(ret->service, cur_test_case.want_result->service)
          << "Test " << cur_test_case.name << " failed, index " << ii;
      EXPECT_EQ(ret->path, cur_test_case.want_result->path)
          << "Test " << cur_test_case.name << " failed, index " << ii;
      EXPECT_EQ(ret->query, cur_test_case.want_result->query)
          << "Test " << cur_test_case.name << " failed, index " << ii;
      EXPECT_EQ(ret->fragment, cur_test_case.want_result->fragment)
          << "Test " << cur_test_case.name << " failed, index " << ii;
    }
  }
}

TEST(URL_PARSER_TEST, JoinUrl) {
  TestJoinUrl<std::string>();
  TestJoinUrl<std::string_view>();
}

TEST(URL_PARSER_TEST, ParseUrl) {
  TestParseUrl<std::string>();
  TestParseUrl<std::string_view>();
}

}  // namespace aimrt::common::util