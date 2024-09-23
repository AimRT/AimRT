// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "zenoh_plugin/global.h"

namespace aimrt::plugins::zenoh_plugin {

aimrt::logger::LoggerRef global_logger;

void SetLogger(aimrt::logger::LoggerRef logger) { global_logger = logger; }
aimrt::logger::LoggerRef GetLogger() {
  return global_logger ? global_logger : aimrt::logger::GetSimpleLoggerRef();
}

}  // namespace aimrt::plugins::zenoh_plugin
