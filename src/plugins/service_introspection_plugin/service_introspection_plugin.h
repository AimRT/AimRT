// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"

namespace aimrt::plugins::service_introspection_plugin {

class ServiceIntrospectionPlugin : public AimRTCorePluginBase {
 public:
  struct Options {
  };

 public:
  ServiceIntrospectionPlugin() = default;
  ~ServiceIntrospectionPlugin() override = default;

  std::string_view Name() const noexcept override { return "service_introspection_plugin"; }

  bool Initialize(runtime::core::AimRTCore *core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void SetPluginLogger();

 private:
  runtime::core::AimRTCore *core_ptr_ = nullptr;

  Options options_;

  bool init_flag_ = false;

  std::atomic_bool stop_flag_ = false;
};

}  // namespace aimrt::plugins::service_introspection_plugin
