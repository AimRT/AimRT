// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/util/version.h"

namespace aimrt::runtime::core::util {

constexpr const char* GetAimRTVersion() {
  return AIMRT_RUNTIME_VERSION;
}
}  // namespace aimrt::runtime::core::util
