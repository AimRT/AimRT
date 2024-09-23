// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <optional>
#include <thread>

#include "aimrt_core_plugin_interface/aimrt_core_plugin_base.h"
#include "net/asio_http_cli.h"
#include "net/asio_http_svr.h"
#include "net/asio_tcp_cli.h"
#include "net/asio_tcp_svr.h"
#include "net/asio_tools.h"
#include "net/asio_udp_cli.h"
#include "net/asio_udp_svr.h"
#include "net_plugin/msg_handle_registry.h"

namespace aimrt::plugins::net_plugin {

class NetPlugin : public AimRTCorePluginBase {
 public:
  struct Options {
    uint32_t thread_num = 2;

    struct HttpOptions {
      std::string listen_ip = "0.0.0.0";
      uint16_t listen_port;
    };
    std::optional<HttpOptions> http_options;

    struct TcpOptions {
      std::string listen_ip = "0.0.0.0";
      uint16_t listen_port;
    };
    std::optional<TcpOptions> tcp_options;

    struct UdpOptions {
      std::string listen_ip = "0.0.0.0";
      uint16_t listen_port;
      uint32_t max_pkg_size = 1024;
    };
    std::optional<UdpOptions> udp_options;
  };

 public:
  NetPlugin() = default;
  ~NetPlugin() override = default;

  std::string_view Name() const noexcept override { return "net_plugin"; }

  bool Initialize(runtime::core::AimRTCore* core_ptr) noexcept override;
  void Shutdown() noexcept override;

 private:
  void SetPluginLogger();
  void RegisterHttpRpcBackend();
  void RegisterHttpChannelBackend();
  void RegisterTcpChannelBackend();
  void RegisterUdpChannelBackend();

 private:
  runtime::core::AimRTCore* core_ptr_ = nullptr;

  Options options_;

  bool init_flag_ = false;

  std::shared_ptr<runtime::common::net::AsioExecutor> asio_executor_ptr_;

  std::shared_ptr<runtime::common::net::AsioHttpClientPool> http_cli_pool_ptr_;
  std::shared_ptr<runtime::common::net::AsioHttpServer> http_svr_ptr_;

  std::shared_ptr<runtime::common::net::AsioTcpClientPool> tcp_cli_pool_ptr_;
  std::shared_ptr<MsgHandleRegistry<boost::asio::ip::tcp::endpoint>> tcp_msg_handle_registry_ptr_;
  std::shared_ptr<runtime::common::net::AsioTcpServer> tcp_svr_ptr_;

  std::shared_ptr<runtime::common::net::AsioUdpClientPool> udp_cli_pool_ptr_;
  std::shared_ptr<MsgHandleRegistry<boost::asio::ip::udp::endpoint>> udp_msg_handle_registry_ptr_;
  std::shared_ptr<runtime::common::net::AsioUdpServer> udp_svr_ptr_;
};

}  // namespace aimrt::plugins::net_plugin
