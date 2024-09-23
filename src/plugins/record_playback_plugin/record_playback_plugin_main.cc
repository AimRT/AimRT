// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_core_plugin_interface/aimrt_core_plugin_main.h"
#include "record_playback_plugin/record_playback_plugin.h"

extern "C" {

aimrt::AimRTCorePluginBase* AimRTDynlibCreateCorePluginHandle() {
  return new aimrt::plugins::record_playback_plugin::RecordPlaybackPlugin();
}

void AimRTDynlibDestroyCorePluginHandle(const aimrt::AimRTCorePluginBase* plugin) {
  delete plugin;
}
}
