// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "core/rpc/rpc_backend_base.h"
#include "net/asio_http_cli.h"
#include "net/asio_http_svr.h"

namespace aimrt::plugins::net_plugin {

class HttpRpcBackend : public runtime::core::rpc::RpcBackendBase {
 public:
  struct Options {
    struct ClientOptions {
      std::string func_name;
      std::string server_url;
    };
    std::vector<ClientOptions> clients_options;

    struct ServerOptions {
      std::string func_name;
    };
    std::vector<ServerOptions> servers_options;
  };

 public:
  HttpRpcBackend(
      const std::shared_ptr<boost::asio::io_context>& io_ptr,
      const std::shared_ptr<runtime::common::net::AsioHttpClientPool>& http_cli_pool_ptr,
      const std::shared_ptr<runtime::common::net::AsioHttpServer>& http_svr_ptr)
      : io_ptr_(io_ptr),
        http_cli_pool_ptr_(http_cli_pool_ptr),
        http_svr_ptr_(http_svr_ptr) {}

  ~HttpRpcBackend() override = default;

  std::string_view Name() const noexcept override { return "http"; }

  void Initialize(YAML::Node options_node) override;
  void Start() override;
  void Shutdown() override;

  void SetRpcRegistry(const runtime::core::rpc::RpcRegistry* rpc_registry_ptr) noexcept override {
    rpc_registry_ptr_ = rpc_registry_ptr;
  }

  bool RegisterServiceFunc(
      const runtime::core::rpc::ServiceFuncWrapper& service_func_wrapper) noexcept override;
  bool RegisterClientFunc(
      const runtime::core::rpc::ClientFuncWrapper& client_func_wrapper) noexcept override;
  void Invoke(
      const std::shared_ptr<runtime::core::rpc::InvokeWrapper>& client_invoke_wrapper_ptr) noexcept override;

 private:
  static std::string_view GetRealFuncName(std::string_view func_name) {
    if (func_name.substr(0, 5) == "ros2:") return func_name.substr(5);
    if (func_name.substr(0, 3) == "pb:") return func_name.substr(3);
    return func_name;
  }

 private:
  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

  Options options_;
  std::atomic<State> state_ = State::kPreInit;

  const runtime::core::rpc::RpcRegistry* rpc_registry_ptr_ = nullptr;

  std::unordered_map<std::string_view, std::string_view> client_server_url_map_;

  std::shared_ptr<boost::asio::io_context> io_ptr_;
  std::shared_ptr<runtime::common::net::AsioHttpClientPool> http_cli_pool_ptr_;
  std::shared_ptr<runtime::common::net::AsioHttpServer> http_svr_ptr_;
};

}  // namespace aimrt::plugins::net_plugin