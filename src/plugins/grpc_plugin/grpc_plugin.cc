// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "grpc_plugin/grpc_plugin.h"

#include <memory>

#include "boost/asio/ip/address_v4.hpp"
#include "core/aimrt_core.h"
#include "core/rpc/rpc_backend_base.h"
#include "grpc_plugin/global.h"
#include "grpc_plugin/grpc_rpc_backend.h"
#include "grpc_plugin/server/server.h"
#include "net/asio_tools.h"
#include "util/log_util.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::grpc_plugin::GrpcPlugin::Options> {
  using Options = aimrt::plugins::grpc_plugin::GrpcPlugin::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["thread_num"] = rhs.thread_num;
    node["listen_ip"] = rhs.listen_ip;
    node["listen_port"] = rhs.listen_port;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["thread_num"])
      rhs.thread_num = node["thread_num"].as<uint32_t>();

    if (node["listen_ip"])
      rhs.listen_ip = node["listen_ip"].as<std::string>();

    rhs.listen_port = node["listen_port"].as<uint16_t>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::grpc_plugin {

auto WrapAimRTLoggerRef(aimrt::logger::LoggerRef logger_ref)
    -> std::shared_ptr<aimrt::common::util::LoggerWrapper> {
  return std::make_shared<aimrt::common::util::LoggerWrapper>(
      aimrt::common::util::LoggerWrapper{
          .get_log_level_func = [logger_ref]() -> uint32_t {
            return logger_ref.GetLogLevel();
          },
          .log_func = [logger_ref](uint32_t lvl,
                                   uint32_t line,
                                   uint32_t column,
                                   const char* file_name,
                                   const char* function_name,
                                   const char* log_data,
                                   size_t log_data_size) {
            logger_ref.Log(
                lvl, line, column, file_name, function_name, log_data, log_data_size);  //
          }});
}

bool GrpcPlugin::Initialize(runtime::core::AimRTCore* core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;

    YAML::Node plugin_options_node = core_ptr_->GetPluginManager().GetPluginOptionsNode(Name());

    if (plugin_options_node && !plugin_options_node.IsNull()) {
      options_ = plugin_options_node.as<Options>();
    }

    init_flag_ = true;

    asio_executor_ptr_ = std::make_shared<aimrt::common::net::AsioExecutor>(options_.thread_num);

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostInitLog,
                                [this] { SetPluginLogger(); });

    http2_svr_ptr_ = std::make_shared<server::AsioHttp2Server>(asio_executor_ptr_->IO());
    http2_cli_pool_ptr_ = std::make_shared<client::AsioHttp2ClientPool>(asio_executor_ptr_->IO());

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitRpc,
                                [this] { RegisterGrpcRpcBackend(); });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreStart, [this] {
      http2_cli_pool_ptr_->SetLogger(WrapAimRTLoggerRef(GetLogger()));
      http2_cli_pool_ptr_->Initialize(client::ClientPoolOptions{.max_client_num = 100});
      http2_cli_pool_ptr_->Start();

      http2_svr_ptr_->SetLogger(WrapAimRTLoggerRef(GetLogger()));
      http2_svr_ptr_->Initialize(server::ServerOptions{
          .ep = {boost::asio::ip::make_address_v4(options_.listen_ip),
                 options_.listen_port},
          .http2_settings = {
              .max_concurrent_streams = 100,
              .initial_window_size = (1U << 31) - 1,
          }});
      http2_svr_ptr_->Start();
    });

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostShutdown, [this] {
      http2_cli_pool_ptr_->Shutdown();
      http2_svr_ptr_->Shutdown();
    });

    asio_executor_ptr_->Start();

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPostShutdown, [this] {
      asio_executor_ptr_->Shutdown();
      asio_executor_ptr_->Join();
    });

    plugin_options_node = options_;
    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Initialize failed, {}", e.what());
  }

  return false;
}

void GrpcPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

    if (http2_cli_pool_ptr_) {
      http2_cli_pool_ptr_->Shutdown();
    }

    if (http2_svr_ptr_) {
      http2_svr_ptr_->Shutdown();
    }

    if (asio_executor_ptr_) {
      asio_executor_ptr_->Shutdown();
      asio_executor_ptr_->Join();
    }

  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

void GrpcPlugin::SetPluginLogger() {
  SetLogger(aimrt::logger::LoggerRef(core_ptr_->GetLoggerManager().GetLoggerProxy().NativeHandle()));
}

void GrpcPlugin::RegisterGrpcRpcBackend() {
  std::unique_ptr<runtime::core::rpc::RpcBackendBase> grpc_rpc_backend_ptr =
      std::make_unique<GrpcRpcBackend>(asio_executor_ptr_->IO(),
                                       http2_svr_ptr_,
                                       http2_cli_pool_ptr_);

  core_ptr_->GetRpcManager().RegisterRpcBackend(std::move(grpc_rpc_backend_ptr));
}

}  // namespace aimrt::plugins::grpc_plugin
