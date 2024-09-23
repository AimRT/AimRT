// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>

#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "parameter_plugin/service.h"

namespace aimrt::plugins::parameter_plugin {

class ParameterPlugin : public AimRTCorePluginBase {
 public:
  struct Options {};

 public:
  ParameterPlugin() = default;
  ~ParameterPlugin() override = default;

  std::string_view Name() const noexcept override { return "parameter_plugin"; }

  bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void SetPluginLogger();
  void RegisterRpcService();

 private:
  runtime::core::AimRTCore* core_ptr_ = nullptr;

  Options options_;

  bool init_flag_ = false;

  std::unique_ptr<ParameterServiceImpl> service_ptr_;
};

}  // namespace aimrt::plugins::parameter_plugin
