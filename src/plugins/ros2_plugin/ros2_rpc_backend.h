// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "core/rpc/rpc_backend_base.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_plugin/global.h"
#include "ros2_plugin/ros2_adapter_rpc_client.h"
#include "ros2_plugin/ros2_adapter_rpc_server.h"
#include "ros2_plugin/ros2_name_encode.h"

namespace aimrt::plugins::ros2_plugin {

class Ros2RpcBackend : public runtime::core::rpc::RpcBackendBase {
 public:
  struct Options {
    struct QosOptions {
      /**
       * @brief history options
       * @param keep_last: Keep the most recent records (cache up to N records, which can be configured through the queue length option)
       * @param keep_all: Keep all records (caches all records, but is limited to the maximum resource that can be configured by the underlying middleware)
       * @param default: System default
       */
      std::string history = "default";

      /**
       * @brief queue depth option (only used with Keep_last)
       */
      int32_t depth = 10;

      /**
       * @brief Reliability Options
       * @param reliable: Reliable (when the message is lost, it will be resent and retransmitted repeatedly to ensure the successful data transmission)
       * @param best_effort: Do your best (try to transfer data but do not guarantee successful transmission, data may be lost when the network is unstable)
       * @param default: System default
       */
      std::string reliability = "default";

      /**
       * @brief queue depth option (only used with Keep_last)
       * @param transient_local: the publisher retains data for late-joining subscribers
       * @param volatile: no data is retained
       * @param default: System default
       */
      std::string durability = "default";

      /**
       * @brief The maximum amount of time expected between subsequent messages posted to the topic
       * @param ms Millisecond timestamp. -1 indicates that this parameter is disabled
       */
      int64_t deadline = -1;

      /**
       * @brief The maximum amount of time between message being published and received without treating the message as stale or expired (expired messages are silently discarded and never actually received).
       * @param ms Millisecond timestamp. -1 indicates that this parameter is disabled
       */
      int64_t lifespan = -1;

      /**
       * @brief How to determine if the publisher is active
       * @param automatic: Automatic (ROS2 will judge based on the time interval between message publishing and receiving)
       * @param manual_by_topic: Publisher needs to declare regularly
       * @param default: System default
       */
      std::string liveliness = "default";

      /**
       * @brief The duration of the active lease period, if the publisher does not declare active beyond this time, it is considered inactive.
       * @param ms Millisecond timestamp. -1 indicates that this parameter is disabled
       */
      int64_t liveliness_lease_duration = -1;
    };

    std::string timeout_executor;

    struct ClientOptions {
      std::string func_name;
      QosOptions qos;
      std::string remapping_rule;
    };
    std::vector<ClientOptions> clients_options;

    struct ServerOptions {
      std::string func_name;
      QosOptions qos;
      std::string remapping_rule;
    };
    std::vector<ServerOptions> servers_options;
  };

 public:
  Ros2RpcBackend() = default;
  ~Ros2RpcBackend() override = default;

  std::string_view Name() const noexcept override { return "ros2"; }

  void Initialize(YAML::Node options_node) override;
  void Start() override;
  void Shutdown() override;

  std::list<std::pair<std::string, std::string>> GenInitializationReport() const noexcept override;

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

  void SetNodePtr(const std::shared_ptr<rclcpp::Node>& ros2_node_ptr) {
    ros2_node_ptr_ = ros2_node_ptr;
  }

  static std::string GetRemappedFuncName(const std::string& input_string, const std::string& matching_rule, const std::string& remapping_rule);

 private:
  static bool CheckRosFunc(std::string_view func_name) {
    return (func_name.substr(0, 5) == "ros2:");
  }

  std::string GetRealRosFuncName(std::string_view func_name) {
    auto find_itr = func_name.find(':');
    AIMRT_CHECK_ERROR_THROW(find_itr != std::string::npos, "Input string does not contain delimiter: ':' ");
    std::string ros_func_name = Ros2NameEncode(func_name.substr(find_itr + 1));
    return rclcpp::extend_name_with_sub_namespace(ros_func_name, ros2_node_ptr_->get_sub_namespace());
  }

 private:
  rclcpp::QoS GetQos(const Options::QosOptions& qos_option);

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
  executor::ExecutorRef timeout_executor_;

  std::shared_ptr<rclcpp::Node> ros2_node_ptr_;

  std::unordered_map<std::string_view, std::shared_ptr<Ros2AdapterServer>> ros2_adapter_server_map_;
  std::unordered_map<std::string_view, std::shared_ptr<Ros2AdapterClient>> ros2_adapter_client_map_;

  std::unordered_map<std::string_view, std::shared_ptr<Ros2AdapterWrapperServer>> ros2_adapter_wrapper_server_map_;
  std::unordered_map<std::string_view, std::shared_ptr<Ros2AdapterWrapperClient>> ros2_adapter_wrapper_client_map_;
};

}  // namespace aimrt::plugins::ros2_plugin
