// Copyright (c) 2024, SmartCar Project
// Iceoryx2 Plugin for AimRT

#include "aimrt_core_plugin_interface/aimrt_core_plugin_main.h"
#include "iceoryx2_plugin/iceoryx2_plugin.h"

extern "C" {

aimrt::AimRTCorePluginBase* AimRTDynlibCreateCorePluginHandle() {
  return new aimrt::plugins::iceoryx2_plugin::Iceoryx2Plugin();
}

void AimRTDynlibDestroyCorePluginHandle(const aimrt::AimRTCorePluginBase* plugin) {
  delete plugin;
}
}
