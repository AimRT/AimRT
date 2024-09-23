// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_core_plugin_interface/aimrt_core_plugin_main.h"
#include "ros2_plugin/ros2_plugin.h"

extern "C" {

aimrt::AimRTCorePluginBase* AimRTDynlibCreateCorePluginHandle() {
  return new aimrt::plugins::ros2_plugin::Ros2Plugin();
}

void AimRTDynlibDestroyCorePluginHandle(const aimrt::AimRTCorePluginBase* plugin) {
  delete plugin;
}
}
