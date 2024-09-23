// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstring>

#include "core/logger/logger_backend_base.h"

namespace aimrt::runtime::core::logger {

class LogLevelTool {
 public:
  static std::string_view GetLogLevelName(aimrt_log_level_t lvl) {
    return kLvlNameArray[static_cast<uint32_t>(lvl)];
  }

  static aimrt_log_level_t GetLogLevelFromName(std::string_view lvl_name) {
    for (int ii = 0; ii <= aimrt_log_level_t::AIMRT_LOG_LEVEL_OFF; ++ii) {
      if (lvl_name.size() != kLvlNameArray[ii].size()) {
        continue;
      }
#if defined(_WIN32)
      if (_strnicmp(lvl_name.data(), kLvlNameArray[ii].data(), lvl_name.size()) == 0)
        return static_cast<aimrt_log_level_t>(ii);
#else
      if (strncasecmp(lvl_name.data(), kLvlNameArray[ii].data(), lvl_name.size()) == 0)
        return static_cast<aimrt_log_level_t>(ii);
#endif
    }
    return aimrt_log_level_t::AIMRT_LOG_LEVEL_OFF;
  }

 private:
  static constexpr std::string_view
      kLvlNameArray[aimrt_log_level_t::AIMRT_LOG_LEVEL_OFF + 1] = {
          "Trace", "Debug", "Info", "Warn", "Error", "Fatal", "Off"};
};
}  // namespace aimrt::runtime::core::logger
