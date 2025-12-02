// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>

#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "aimrt_module_cpp_interface/channel/channel_handle.h"

namespace aimrt::plugins::service_introspection_plugin {

class ServiceIntrospectionPlugin : public AimRTCorePluginBase {
 public:
  struct Options {
    enum class Mode {
      kMeta,
      khybrid,
      kFull
    };

    Mode mode = Mode::kFull;

    enum class RpcSerializationType {
      kJson,
      kAuto,
    };
    RpcSerializationType rpc_serialization_type = RpcSerializationType::kJson;

    std::string client_info_topic_name;
    std::string server_info_topic_name;
  };

 public:
  ServiceIntrospectionPlugin() = default;
  ~ServiceIntrospectionPlugin() override = default;

  std::string_view Name() const noexcept override { return "service_introspection_plugin"; }

  bool Initialize(runtime::core::AimRTCore *core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void RegisterPublisher();
  void SetPluginLogger();
  void RegisterClientFilter();
  void RegisterServiceFilter();

 private:
  runtime::core::AimRTCore *core_ptr_ = nullptr;

  Options options_;

  aimrt::channel::PublisherRef publisher_;
  aimrt::channel::PublisherRef publisher2_;

  std::unique_ptr<aimrt::channel::PublisherRef> client_info_publisher_;
  std::unique_ptr<aimrt::channel::PublisherRef> server_info_publisher_;

  bool init_flag_ = false;
  std::atomic_bool stop_flag_ = false;
};

}  // namespace aimrt::plugins::service_introspection_plugin
