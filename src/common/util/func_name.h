// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include <string_view>

namespace aimrt::common::util {

/**
 * @brief Get AimRT function name without prefix
 * @param func_name AimRT function name
 * @return real function name
 */
std::string_view GetAimRTFuncNameWithoutPrefix(std::string_view func_name) {
  if (func_name.substr(0, 3) == "pb:") return func_name.substr(3);
  if (func_name.substr(0, 5) == "ros2:") return func_name.substr(5);
  return func_name;
}

}  // namespace aimrt::common::util
