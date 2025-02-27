// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <string_view>

namespace aimrt::common::util {

/**
 * @brief Url structure
 *
 * @tparam StringType
 */
template <class StringType = std::string_view>
  requires(std::is_same_v<StringType, std::string_view> ||
           std::is_same_v<StringType, std::string>)
struct Url {
  StringType protocol;
  StringType host;
  StringType service;  // port
  StringType path;
  StringType query;
  StringType fragment;
};

/**
 * @brief Parse a url
 * @note [protocol://][host][:service][path][?query][#fragment]
 * @param url_str url string
 * @return std::optional<UrlView> url structure
 */
template <class StringType = std::string_view>
std::optional<Url<StringType>> ParseUrl(std::string_view url_str) {
  std::regex url_regex(
      R"(^(([^:\/?#]+)://)?(([^\/?#]*))?([^?#]*)(\?([^#]*))?(#(.*))?)",
      std::regex::ECMAScript);
  std::match_results<std::string_view::const_iterator> url_match_result;

  if (!std::regex_match(url_str.begin(), url_str.end(), url_match_result, url_regex))
    return std::nullopt;

  Url<StringType> url;
  if (url_match_result[2].matched)
    url.protocol = StringType(url_match_result[2].first, url_match_result[2].second);

  if (url_match_result[4].matched) {
    StringType auth(url_match_result[4].first, url_match_result[4].second);
    size_t pos = auth.find_first_of(':');
    if (pos != StringType::npos) {
      url.host = auth.substr(0, pos);
      url.service = auth.substr(pos + 1);
    } else {
      url.host = auth;
    }
  }

  if (url_match_result[5].matched)
    url.path = StringType(url_match_result[5].first, url_match_result[5].second);

  if (url_match_result[7].matched)
    url.query = StringType(url_match_result[7].first, url_match_result[7].second);

  if (url_match_result[9].matched)
    url.fragment = StringType(url_match_result[9].first, url_match_result[9].second);

  return std::optional<Url<StringType>>{url};
}

/**
 * @brief Join a url
 * @note [protocol://][host][:service][path][?query][#fragment]
 * @param url url structure
 * @return std::string url string
 */
template <class StringType = std::string_view>
std::string JoinUrl(const Url<StringType>& url) {
  std::stringstream ss;

  if (!url.protocol.empty()) ss << url.protocol << "://";
  if (!url.host.empty()) ss << url.host;
  if (!url.service.empty()) ss << ":" << url.service;
  if (!url.path.empty()) {
    if (url.path[0] != '/') ss << '/';
    ss << url.path;
  }
  if (!url.query.empty()) ss << '?' << url.query;
  if (!url.fragment.empty()) ss << '#' << url.fragment;

  return ss.str();
}

}  // namespace aimrt::common::util