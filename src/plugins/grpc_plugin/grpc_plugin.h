// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>
#include <memory>

#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "grpc_plugin/client/client_pool.h"
#include "grpc_plugin/server/server.h"
#include "net/asio_tools.h"

namespace aimrt::plugins::grpc_plugin {

class GrpcPlugin : public AimRTCorePluginBase {
 public:
  struct Options {
    uint32_t thread_num = 2;

    std::string listen_ip = "0.0.0.0";
    uint16_t listen_port = 50051;
    // Add configurable timeout with a default value of 5 seconds
    std::chrono::seconds timeout = std::chrono::seconds(5);
  };

 public:
  GrpcPlugin() = default;
  ~GrpcPlugin() override = default;

  [[nodiscard]] std::string_view Name() const noexcept override { return "grpc_plugin"; }

  bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void SetPluginLogger();
  void RegisterGrpcRpcBackend();

 private:
  runtime::core::AimRTCore* core_ptr_ = nullptr;

  Options options_;

  bool init_flag_ = false;

  std::shared_ptr<aimrt::common::net::AsioExecutor> asio_executor_ptr_;

  std::shared_ptr<plugins::grpc_plugin::server::AsioHttp2Server> http2_svr_ptr_;
  std::shared_ptr<plugins::grpc_plugin::client::AsioHttp2ClientPool> http2_cli_pool_ptr_;
};

}  // namespace aimrt::plugins::grpc_plugin
