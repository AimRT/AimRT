// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>

#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "time_manipulator_plugin/service.h"

namespace aimrt::plugins::time_manipulator_plugin {

class TimeManipulatorPlugin : public AimRTCorePluginBase {
 public:
  struct Options {};

 public:
  TimeManipulatorPlugin() = default;
  ~TimeManipulatorPlugin() override = default;

  std::string_view Name() const noexcept override { return "time_manipulator_plugin"; }

  bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void SetPluginLogger();
  void RegisterTimeManipulatorExecutor();
  void RegisterRpcService();

 private:
  runtime::core::AimRTCore* core_ptr_ = nullptr;

  Options options_;

  bool init_flag_ = false;

  std::unique_ptr<TimeManipulatorServiceImpl> service_ptr_;
};

}  // namespace aimrt::plugins::time_manipulator_plugin
