// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <chrono>
#include <string_view>

#include "aimrt_module_c_interface/logger/logger_base.h"

namespace aimrt::runtime::core::logger {

struct LogDataWrapper {
  std::string_view module_name;
  size_t thread_id;
  std::chrono::system_clock::time_point t;
  aimrt_log_level_t lvl;
  uint32_t line;
  uint32_t column;
  const char* file_name;
  const char* function_name;
  const char* log_data;
  size_t log_data_size;
};

}  // namespace aimrt::runtime::core::logger