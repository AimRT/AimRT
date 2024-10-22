// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "core/rpc/rpc_backend_base.h"

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "ros2_plugin/ros2_adapter_rpc_client.h"
#include "ros2_plugin/ros2_adapter_rpc_server.h"

#include "rclcpp/rclcpp.hpp"

namespace aimrt::plugins::ros2_plugin {

class Ros2RpcBackend : public runtime::core::rpc::RpcBackendBase {
 public:
  struct Options {
    struct QosOptions {
      /**
       * @brief 历史记录选项
       * @param keep_last:保留最近的记录(缓存最多N条记录，可通过队列长度选项来配置)
       * @param keep_all:保留所有记录(缓存所有记录，但受限于底层中间件可配置的最大资源)
       * @param default:系统默认
       */
      std::string history = "default";

      /**
       * @brief 队列深度选项(只能与Keep_last配合使用)
       */
      int32_t depth = 10;

      /**
       * @brief 可靠性选项
       * @param reliable:可靠的(消息丢失时，会重新发送,反复重传以保证数据传输成功)
       * @param best_effort:尽力而为的(尝试传输数据但不保证成功传输,当网络不稳定时可能丢失数据)
       * @param default:系统默认
       */
      std::string reliability = "default";

      /**
       * @brief 持续性选项
       * @param transient_local:局部瞬态(发布器为晚连接(late-joining)的订阅器保留数据)
       * @param volatile:易变态(不保留任何数据)
       * @param default:系统默认
       */
      std::string durability = "default";

      /**
       * @brief 后续消息发布到主题之间的预期最大时间量
       * @param ms级时间戳 -1为不设置
       */
      int64_t deadline = -1;

      /**
       * @brief 消息发布和接收之间的最大时间量，而不将消息视为陈旧或过期（过期的消息被静默地丢弃，并且实际上从未被接收）。
       * @param ms级时间戳 -1为不设置
       */
      int64_t lifespan = -1;

      /**
       * @brief 如何确定发布者是否活跃
       * @param automatic:自动(ROS2会根据消息发布和接收的时间间隔来判断)
       * @param manual_by_topic:需要发布者定期声明
       * @param default:系统默认
       */
      std::string liveliness = "default";

      /**
       * @brief 活跃性租期的时长，如果超过这个时间发布者没有声明活跃，则被认为是不活跃的。
       * @param ms级时间戳 -1为不设置
       */
      int64_t liveliness_lease_duration = -1;
    };

    struct RemappingOptions {
      std::string matching_rule;
      std::string replacement_rule;
    };

    std::string timeout_executor;

    struct ClientOptions {
      std::string func_name;
      QosOptions qos;
      RemappingOptions remapping;
    };
    std::vector<ClientOptions> clients_options;

    struct ServerOptions {
      std::string func_name;
      QosOptions qos;
      RemappingOptions remapping;
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

 private:
  static bool CheckRosFunc(std::string_view func_name) {
    return (func_name.substr(0, 5) == "ros2:");
  }

  std::string GetRealRosFuncName(std::string_view func_name) {
    return rclcpp::extend_name_with_sub_namespace(
        std::string(func_name.substr(5)), ros2_node_ptr_->get_sub_namespace());
  }

 private:
  rclcpp::QoS GetQos(const Options::QosOptions& qos_option);

  std::string GetRemappedFuncName(const std::string& func_name, const Options::RemappingOptions& remapping_option);

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
