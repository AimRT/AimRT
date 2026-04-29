// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "agi_header_plugin/agi_header_plugin.h"
#include "aimrt_core_plugin_interface/aimrt_core_plugin_main.h"

extern "C" {

aimrt::AimRTCorePluginBase* AimRTDynlibCreateCorePluginHandle() {
  return new aimrt::plugins::agi_header_plugin::AgiHeaderPlugin();
}

void AimRTDynlibDestroyCorePluginHandle(const aimrt::AimRTCorePluginBase* plugin) {
  delete plugin;
}

}  // extern "C"
