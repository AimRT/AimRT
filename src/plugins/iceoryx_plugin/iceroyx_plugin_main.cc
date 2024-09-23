// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_core_plugin_interface/aimrt_core_plugin_main.h"
#include "iceoryx_plugin/iceoryx_plugin.h"

extern "C" {

aimrt::AimRTCorePluginBase* AimRTDynlibCreateCorePluginHandle() {
  return new aimrt::plugins::iceoryx_plugin::IceoryxPlugin();
}

void AimRTDynlibDestroyCorePluginHandle(const aimrt::AimRTCorePluginBase* plugin) {
  delete plugin;
}
}
