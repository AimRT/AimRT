// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "net_plugin/net_plugin.h"

#include "core/aimrt_core.h"
#include "net_plugin/global.h"
#include "net_plugin/http/http_channel_backend.h"
#include "net_plugin/http/http_rpc_backend.h"
#include "net_plugin/tcp/tcp_channel_backend.h"
#include "net_plugin/udp/udp_channel_backend.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::net_plugin::NetPlugin::Options> {
  using Options = aimrt::plugins::net_plugin::NetPlugin::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["thread_num"] = rhs.thread_num;

    if (rhs.http_options) {
      node["http_options"] = Node();
      node["http_options"]["listen_ip"] = rhs.http_options->listen_ip;
      node["http_options"]["listen_port"] = rhs.http_options->listen_port;
    }

    if (rhs.tcp_options) {
      node["tcp_options"] = Node();
      node["tcp_options"]["listen_ip"] = rhs.tcp_options->listen_ip;
      node["tcp_options"]["listen_port"] = rhs.tcp_options->listen_port;
    }

    if (rhs.udp_options) {
      node["udp_options"] = Node();
      node["udp_options"]["listen_ip"] = rhs.udp_options->listen_ip;
      node["udp_options"]["listen_port"] = rhs.udp_options->listen_port;
      node["udp_options"]["max_pkg_size"] = rhs.udp_options->max_pkg_size;
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    rhs.thread_num = node["thread_num"].as<uint32_t>();

    if (node["http_options"]) {
      rhs.http_options = Options::HttpOptions{
          .listen_ip = node["http_options"]["listen_ip"].as<std::string>(),
          .listen_port = node["http_options"]["listen_port"].as<uint16_t>(),
      };
    }

    if (node["tcp_options"]) {
      rhs.tcp_options = Options::TcpOptions{
          .listen_ip = node["tcp_options"]["listen_ip"].as<std::string>(),
          .listen_port = node["tcp_options"]["listen_port"].as<uint16_t>(),
      };
    }

    if (node["udp_options"]) {
      rhs.udp_options = Options::UdpOptions{
          .listen_ip = node["udp_options"]["listen_ip"].as<std::string>(),
          .listen_port = node["udp_options"]["listen_port"].as<uint16_t>(),
      };
      if (node["udp_options"]["max_pkg_size"]) {
        rhs.udp_options->max_pkg_size = node["udp_options"]["max_pkg_size"].as<uint32_t>();
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::net_plugin {

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

bool NetPlugin::Initialize(runtime::core::AimRTCore* core_ptr) noexcept {
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

    // http
    if (options_.http_options) {
      http_cli_pool_ptr_ = std::make_shared<aimrt::common::net::AsioHttpClientPool>(asio_executor_ptr_->IO());
      http_svr_ptr_ = std::make_shared<aimrt::common::net::AsioHttpServer>(asio_executor_ptr_->IO());

      core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitRpc,
                                  [this] { RegisterHttpRpcBackend(); });

      core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitChannel,
                                  [this] { RegisterHttpChannelBackend(); });

      core_ptr_->RegisterHookFunc(
          runtime::core::AimRTCore::State::kPreStart,
          [this] {
            http_cli_pool_ptr_->SetLogger(WrapAimRTLoggerRef(GetLogger()));
            http_cli_pool_ptr_->Initialize(aimrt::common::net::AsioHttpClientPool::Options{});
            http_cli_pool_ptr_->Start();

            http_svr_ptr_->SetLogger(WrapAimRTLoggerRef(GetLogger()));
            http_svr_ptr_->Initialize(aimrt::common::net::AsioHttpServer::Options{
                .ep = {boost::asio::ip::make_address_v4(options_.http_options->listen_ip),
                       options_.http_options->listen_port}});
            http_svr_ptr_->Start();
          });

      core_ptr_->RegisterHookFunc(
          runtime::core::AimRTCore::State::kPostShutdown,
          [this] {
            http_cli_pool_ptr_->Shutdown();
            http_svr_ptr_->Shutdown();
          });
    }

    // tcp
    if (options_.tcp_options) {
      tcp_cli_pool_ptr_ = std::make_shared<aimrt::common::net::AsioTcpClientPool>(asio_executor_ptr_->IO());
      tcp_msg_handle_registry_ptr_ = std::make_shared<MsgHandleRegistry<boost::asio::ip::tcp::endpoint>>();
      tcp_svr_ptr_ = std::make_shared<aimrt::common::net::AsioTcpServer>(asio_executor_ptr_->IO());

      core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitChannel,
                                  [this] { RegisterTcpChannelBackend(); });

      core_ptr_->RegisterHookFunc(
          runtime::core::AimRTCore::State::kPreStart,
          [this] {
            tcp_cli_pool_ptr_->SetLogger(WrapAimRTLoggerRef(GetLogger()));
            tcp_cli_pool_ptr_->Initialize(aimrt::common::net::AsioTcpClientPool::Options{});
            tcp_cli_pool_ptr_->Start();

            tcp_svr_ptr_->SetLogger(WrapAimRTLoggerRef(GetLogger()));
            tcp_svr_ptr_->RegisterMsgHandle(tcp_msg_handle_registry_ptr_->GetMsgHandleFunc());
            tcp_svr_ptr_->Initialize(aimrt::common::net::AsioTcpServer::Options{
                .ep = {boost::asio::ip::make_address_v4(options_.tcp_options->listen_ip),
                       options_.tcp_options->listen_port}});
            tcp_svr_ptr_->Start();
          });

      core_ptr_->RegisterHookFunc(
          runtime::core::AimRTCore::State::kPostShutdown,
          [this] {
            tcp_cli_pool_ptr_->Shutdown();
            tcp_svr_ptr_->Shutdown();
          });
    }

    // udp
    if (options_.udp_options) {
      udp_cli_pool_ptr_ = std::make_shared<aimrt::common::net::AsioUdpClientPool>(asio_executor_ptr_->IO());
      udp_msg_handle_registry_ptr_ = std::make_shared<MsgHandleRegistry<boost::asio::ip::udp::endpoint>>();
      udp_svr_ptr_ = std::make_shared<aimrt::common::net::AsioUdpServer>(asio_executor_ptr_->IO());

      core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitChannel,
                                  [this] { RegisterUdpChannelBackend(); });

      core_ptr_->RegisterHookFunc(
          runtime::core::AimRTCore::State::kPreStart,
          [this] {
            udp_cli_pool_ptr_->SetLogger(WrapAimRTLoggerRef(GetLogger()));
            udp_cli_pool_ptr_->Initialize(aimrt::common::net::AsioUdpClientPool::Options{});
            udp_cli_pool_ptr_->Start();

            udp_svr_ptr_->SetLogger(WrapAimRTLoggerRef(GetLogger()));
            udp_svr_ptr_->RegisterMsgHandle(udp_msg_handle_registry_ptr_->GetMsgHandleFunc());
            udp_svr_ptr_->Initialize(aimrt::common::net::AsioUdpServer::Options{
                .ep = {boost::asio::ip::make_address_v4(options_.udp_options->listen_ip),
                       options_.udp_options->listen_port},
                .max_package_size = options_.udp_options->max_pkg_size});
            udp_svr_ptr_->Start();
          });

      core_ptr_->RegisterHookFunc(
          runtime::core::AimRTCore::State::kPostShutdown,
          [this] {
            udp_cli_pool_ptr_->Shutdown();
            udp_svr_ptr_->Shutdown();
          });
    }

    asio_executor_ptr_->Start();

    core_ptr_->RegisterHookFunc(
        runtime::core::AimRTCore::State::kPostShutdown,
        [this] {
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

void NetPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

    // tcp
    if (udp_svr_ptr_)
      udp_svr_ptr_->Shutdown();

    if (udp_msg_handle_registry_ptr_)
      udp_msg_handle_registry_ptr_->Shutdown();

    if (udp_cli_pool_ptr_)
      udp_cli_pool_ptr_->Shutdown();

    // tcp
    if (tcp_svr_ptr_)
      tcp_svr_ptr_->Shutdown();

    if (tcp_msg_handle_registry_ptr_)
      tcp_msg_handle_registry_ptr_->Shutdown();

    if (tcp_cli_pool_ptr_)
      tcp_cli_pool_ptr_->Shutdown();

    // http
    if (http_svr_ptr_)
      http_svr_ptr_->Shutdown();

    if (http_cli_pool_ptr_)
      http_cli_pool_ptr_->Shutdown();

    // asio
    if (asio_executor_ptr_) {
      asio_executor_ptr_->Shutdown();
      asio_executor_ptr_->Join();
    }

  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

void NetPlugin::SetPluginLogger() {
  SetLogger(aimrt::logger::LoggerRef(
      core_ptr_->GetLoggerManager().GetLoggerProxy().NativeHandle()));
}

void NetPlugin::RegisterHttpRpcBackend() {
  std::unique_ptr<runtime::core::rpc::RpcBackendBase> http_rpc_backend_ptr =
      std::make_unique<HttpRpcBackend>(asio_executor_ptr_->IO(),
                                       http_cli_pool_ptr_,
                                       http_svr_ptr_);

  core_ptr_->GetRpcManager().RegisterRpcBackend(std::move(http_rpc_backend_ptr));
}

void NetPlugin::RegisterHttpChannelBackend() {
  std::unique_ptr<runtime::core::channel::ChannelBackendBase> http_channel_backend_ptr =
      std::make_unique<HttpChannelBackend>(asio_executor_ptr_->IO(),
                                           http_cli_pool_ptr_,
                                           http_svr_ptr_);

  core_ptr_->GetChannelManager().RegisterChannelBackend(std::move(http_channel_backend_ptr));
}

void NetPlugin::RegisterTcpChannelBackend() {
  std::unique_ptr<runtime::core::channel::ChannelBackendBase> tcp_channel_backend_ptr =
      std::make_unique<TcpChannelBackend>(asio_executor_ptr_->IO(),
                                          tcp_cli_pool_ptr_,
                                          tcp_svr_ptr_,
                                          tcp_msg_handle_registry_ptr_);

  core_ptr_->GetChannelManager().RegisterChannelBackend(std::move(tcp_channel_backend_ptr));
}

void NetPlugin::RegisterUdpChannelBackend() {
  std::unique_ptr<runtime::core::channel::ChannelBackendBase> udp_channel_backend_ptr =
      std::make_unique<UdpChannelBackend>(asio_executor_ptr_->IO(),
                                          udp_cli_pool_ptr_,
                                          udp_svr_ptr_,
                                          udp_msg_handle_registry_ptr_);

  core_ptr_->GetChannelManager().RegisterChannelBackend(std::move(udp_channel_backend_ptr));
}

}  // namespace aimrt::plugins::net_plugin
