// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <utility>

#include "core/rpc/rpc_backend_base.h"
#include "core/util/rpc_client_tool.h"
#include "mqtt_plugin/msg_handle_registry.h"

namespace aimrt::plugins::mqtt_plugin {

class MqttRpcBackend : public runtime::core::rpc::RpcBackendBase {
 public:
  struct Options {
    std::string timeout_executor;

    struct ClientOptions {
      std::string func_name;
      std::string server_mqtt_id;
      int qos = 2;
    };
    std::vector<ClientOptions> clients_options;

    struct ServerOptions {
      std::string func_name;
      bool allow_share = true;
      int qos = 2;
    };
    std::vector<ServerOptions> servers_options;
  };

 public:
  MqttRpcBackend(
      std::string client_id,
      MQTTAsync& client,
      uint32_t max_pkg_size,
      std::shared_ptr<MsgHandleRegistry> msg_handle_registry_ptr)
      : client_id_(std::move(client_id)),
        client_(client),
        max_pkg_size_(max_pkg_size),
        msg_handle_registry_ptr_(std::move(msg_handle_registry_ptr)) {}

  ~MqttRpcBackend() override = default;

  std::string_view Name() const noexcept override { return "mqtt"; }

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

  void RegisterGetExecutorFunc(const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func);

  void SubscribeMqttTopic();
  void UnSubscribeMqttTopic();

 private:
  static std::string_view GetRealFuncName(std::string_view func_name) {
    if (func_name.substr(0, 5) == "ros2:") return func_name.substr(5);
    if (func_name.substr(0, 3) == "pb:") return func_name.substr(3);
    return func_name;
  }

  void ReturnRspWithStatusCode(
      std::string_view mqtt_pub_topic,
      int qos,
      std::string_view serialization_type,
      const char* req_id_buf,
      uint32_t code);

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

  std::function<executor::ExecutorRef(std::string_view)> get_executor_func_;

  std::string client_id_;
  MQTTAsync& client_;
  uint32_t max_pkg_size_;

  struct MqttSubInfo {
    std::string topic;
    int qos;
  };
  std::vector<MqttSubInfo> sub_info_vec_;

  std::shared_ptr<MsgHandleRegistry> msg_handle_registry_ptr_;

  std::atomic_uint32_t req_id_ = 0;

  struct ClientCfgInfo {
    std::string server_mqtt_id;
    int qos;
  };
  std::unordered_map<std::string_view, ClientCfgInfo> client_cfg_info_map_;

  std::unique_ptr<runtime::core::util::RpcClientTool<std::shared_ptr<runtime::core::rpc::InvokeWrapper>>>
      client_tool_ptr_;
};

}  // namespace aimrt::plugins::mqtt_plugin