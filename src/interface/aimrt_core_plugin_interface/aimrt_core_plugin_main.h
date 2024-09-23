// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "aimrt_core_plugin_interface/aimrt_core_plugin_export.h"

extern "C" {

AIMRT_CORE_PLUGIN_EXPORT aimrt::AimRTCorePluginBase* AimRTDynlibCreateCorePluginHandle();

AIMRT_CORE_PLUGIN_EXPORT void AimRTDynlibDestroyCorePluginHandle(const aimrt::AimRTCorePluginBase* plugin);
}
