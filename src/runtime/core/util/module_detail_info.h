// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>

#include "aimrt_module_c_interface/logger/logger_base.h"

namespace aimrt::runtime::core::util {

struct ModuleDetailInfo {
  // base info
  std::string name;
  std::string pkg_path;

  // detail info
  uint32_t major_version = 0;
  uint32_t minor_version = 0;
  uint32_t patch_version = 0;
  uint32_t build_version = 0;

  std::string author;
  std::string description;

  // options
  aimrt_log_level_t log_lvl = aimrt_log_level_t::AIMRT_LOG_LEVEL_TRACE;
  bool use_default_log_lvl = true;
  std::string cfg_file_path;
};

}  // namespace aimrt::runtime::core::util
