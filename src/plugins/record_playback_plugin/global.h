// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/logger/logger.h"

namespace aimrt::plugins::record_playback_plugin {

void SetLogger(aimrt::logger::LoggerRef);
aimrt::logger::LoggerRef GetLogger();

constexpr uint32_t kVersion = 1;

}  // namespace aimrt::plugins::record_playback_plugin
