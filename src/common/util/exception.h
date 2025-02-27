// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <source_location>
#include <string>

#include "util/format.h"

namespace aimrt::common::util {

class AimRTException : public std::exception {
 public:
  template <typename... Args>
    requires std::constructible_from<std::string, Args...>
  explicit AimRTException(Args&&... args)
      : err_msg_(std::forward<Args>(args)...) {}

  template <typename... Args>
    requires std::constructible_from<std::string, Args...>
  explicit AimRTException(const std::source_location& loc, Args&&... args)
      : err_msg_(::aimrt_fmt::format("{}:{} - {}",
                                     loc.file_name(),
                                     loc.line(),
                                     std::string(std::forward<Args>(args)...))) {}

  ~AimRTException() noexcept override = default;

  const char* what() const noexcept override { return err_msg_.c_str(); }

 private:
  std::string err_msg_;
};

}  // namespace aimrt::common::util

#define AIMRT_ASSERT(__expr__, __fmt__, ...)                                                  \
  do {                                                                                        \
    if (!(__expr__)) [[unlikely]] {                                                           \
      throw aimrt::common::util::AimRTException(std::source_location::current(),              \
                                                ::aimrt_fmt::format(__fmt__, ##__VA_ARGS__)); \
    }                                                                                         \
  } while (0)
