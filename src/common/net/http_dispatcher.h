// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <functional>
#include <list>
#include <regex>
#include <string>
#include <string_view>

namespace aimrt::common::net {

/**
 * @brief Http dispatcher
 * @note Use regular expression matching to return the first matching handle according to the order of Register.
 * @tparam HttpHandleFuncType
 */
template <typename HttpHandleFuncType>
class HttpDispatcher {
 public:
  using HttpHandle = std::function<HttpHandleFuncType>;

 public:
  HttpDispatcher() = default;
  ~HttpDispatcher() = default;

  template <typename... Args>
    requires std::constructible_from<HttpHandle, Args...>
  void RegisterHttpHandle(std::string_view pattern, Args&&... args) {
    http_handle_list_.emplace_back(
        std::regex(std::string(pattern), std::regex::ECMAScript | std::regex::icase),
        std::forward<Args>(args)...);
  }

  const HttpHandle& GetHttpHandle(std::string_view path) const {
    for (const auto& itr : http_handle_list_) {
      if (std::regex_match(path.begin(), path.end(), itr.first) && itr.second)
        return itr.second;
    }

    static HttpHandle empty_handle;
    return empty_handle;
  }

 private:
  std::list<std::pair<std::regex, HttpHandle>> http_handle_list_;
};

}  // namespace aimrt::common::net