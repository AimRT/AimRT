// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>

#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "log_control_plugin/service.h"

namespace aimrt::plugins::log_control_plugin {

class LogControlPlugin : public AimRTCorePluginBase {
 public:
  struct Options {};

 public:
  LogControlPlugin() = default;
  ~LogControlPlugin() override = default;

  std::string_view Name() const noexcept override { return "log_control_plugin"; }

  bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void SetPluginLogger();
  void RegisterRpcService();

 private:
  runtime::core::AimRTCore* core_ptr_ = nullptr;

  Options options_;

  bool init_flag_ = false;

  std::unique_ptr<LogControlServiceImpl> service_ptr_;
};

}  // namespace aimrt::plugins::log_control_plugin
