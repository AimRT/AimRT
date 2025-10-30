// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>

#include <source_location>
#include "util/format.h"

namespace aimrt::common::util {

class AimRTException : public std::exception {
 public:
  template <typename... Args>
    requires std::constructible_from<std::string, Args...>
  explicit AimRTException(Args&&... args)
      : err_msg_(std::forward<Args>(args)...) {}

  ~AimRTException() noexcept override = default;

  const char* what() const noexcept override { return err_msg_.c_str(); }

 private:
  std::string err_msg_;
};

class AimRTContextException : public std::exception {
 public:
  explicit AimRTContextException(
      std::string msg,
      std::source_location call_loc = std::source_location::current()) noexcept
      : err_msg_(
            ::aimrt_fmt::format(
                "{}:{}: {}",
                call_loc.file_name(),
                call_loc.line(),
                msg)),
        location_(call_loc) {}

  ~AimRTContextException() noexcept override = default;

  const char* what() const noexcept override { return err_msg_.c_str(); }

  const std::source_location& location() const noexcept { return location_; }
  unsigned int line() const noexcept { return location_.line(); }
  const char* file_name() const noexcept { return location_.file_name(); }

 private:
  std::string err_msg_;
  std::source_location location_;
};

}  // namespace aimrt::common::util

#define AIMRT_ASSERT(__expr__, __fmt__, ...)                                                  \
  do {                                                                                        \
    if (!(__expr__)) [[unlikely]] {                                                           \
      throw aimrt::common::util::AimRTException(::aimrt_fmt::format(__fmt__, ##__VA_ARGS__)); \
    }                                                                                         \
  } while (0)

#define AIMRT_ASSERT_WITH_LOC(__call_loc__, __expr__, __fmt__, ...) \
  do {                                                              \
    if (!(__expr__)) [[unlikely]] {                                 \
      throw aimrt::common::util::AimRTException(                    \
          ::aimrt_fmt::format(                                      \
              "{}:{}: {}",                                          \
              __call_loc__.file_name(),                             \
              __call_loc__.line(),                                  \
              ::aimrt_fmt::format(__fmt__, ##__VA_ARGS__)));        \
    }                                                               \
  } while (0)
