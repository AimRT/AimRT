// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_core_plugin_interface/aimrt_core_plugin_main.h"
#include "log_control_plugin/log_control_plugin.h"

extern "C" {

aimrt::AimRTCorePluginBase* AimRTDynlibCreateCorePluginHandle() {
  return new aimrt::plugins::log_control_plugin::LogControlPlugin();
}

void AimRTDynlibDestroyCorePluginHandle(const aimrt::AimRTCorePluginBase* plugin) {
  delete plugin;
}
}
