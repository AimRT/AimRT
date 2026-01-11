// Copyright (c) 2024, SmartCar Project
// Iceoryx2 Plugin for AimRT

#pragma once

#include "aimrt_module_cpp_interface/logger/logger.h"

namespace aimrt::plugins::iceoryx2_plugin {

void SetLogger(aimrt::logger::LoggerRef);
aimrt::logger::LoggerRef GetLogger();

}  // namespace aimrt::plugins::iceoryx2_plugin
