// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <list>
#include <string>
#include <string_view>

namespace aimrt {

namespace runtime::core {
class AimRTCore;
}

class AimRTCorePluginBase {
 public:
  AimRTCorePluginBase() = default;
  virtual ~AimRTCorePluginBase() = default;

  AimRTCorePluginBase(const AimRTCorePluginBase&) = delete;
  AimRTCorePluginBase& operator=(const AimRTCorePluginBase&) = delete;

  virtual std::string_view Name() const noexcept = 0;

  virtual bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept = 0;
  virtual void Shutdown() noexcept = 0;

  virtual std::list<std::pair<std::string, std::string>> GenInitializationReport() const { return {}; }
};

}  // namespace aimrt
