// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <mutex>
#include <unordered_map>

#include "core/rpc/rpc_backend_base.h"
#include "core/util/rpc_client_tool.h"

#include "rclcpp/client.hpp"
#include "ros2_plugin/global.h"

namespace aimrt::plugins::ros2_plugin {

class Ros2AdapterClient : public rclcpp::ClientBase {
 public:
  Ros2AdapterClient(
      rclcpp::node_interfaces::NodeBaseInterface* node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
      const runtime::core::rpc::ClientFuncWrapper& client_func_wrapper,
      const std::string& real_ros2_func_name,
      const rclcpp::QoS& qos,
      aimrt::executor::ExecutorRef timeout_executor);
  ~Ros2AdapterClient() override = default;

  std::shared_ptr<void> create_response() override;
  std::shared_ptr<rmw_request_id_t> create_request_header() override;
  void handle_response(std::shared_ptr<rmw_request_id_t> request_header,
                       std::shared_ptr<void> response) override;

  void Invoke(
      const std::shared_ptr<runtime::core::rpc::InvokeWrapper>& client_invoke_wrapper_ptr);

  void Start() { run_flag_.store(true); }
  void Shutdown() { run_flag_.store(false); }

 private:
  std::atomic_bool run_flag_ = false;
  const runtime::core::rpc::ClientFuncWrapper& client_func_wrapper_;
  std::string real_ros2_func_name_;

  pthread_mutex_t rpc_client_mutex_;
  runtime::core::util::RpcClientTool<std::shared_ptr<runtime::core::rpc::InvokeWrapper>>
      client_tool_;
};

class Ros2AdapterWrapperClient : public rclcpp::ClientBase {
 public:
  Ros2AdapterWrapperClient(
      rclcpp::node_interfaces::NodeBaseInterface* node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
      const runtime::core::rpc::ClientFuncWrapper& client_func_wrapper,
      const std::string& real_ros2_func_name,
      const rclcpp::QoS& qos,
      aimrt::executor::ExecutorRef timeout_executor);
  ~Ros2AdapterWrapperClient() override = default;

  std::shared_ptr<void> create_response() override;
  std::shared_ptr<rmw_request_id_t> create_request_header() override;
  void handle_response(std::shared_ptr<rmw_request_id_t> request_header,
                       std::shared_ptr<void> response) override;

  void Invoke(
      const std::shared_ptr<runtime::core::rpc::InvokeWrapper>& client_invoke_wrapper_ptr);

  void Start() { run_flag_.store(true); }
  void Shutdown() { run_flag_.store(false); }

 private:
  std::atomic_bool run_flag_ = false;
  const runtime::core::rpc::ClientFuncWrapper& client_func_wrapper_;
  std::string real_ros2_func_name_;

  pthread_mutex_t rpc_client_mutex_;
  runtime::core::util::RpcClientTool<std::shared_ptr<runtime::core::rpc::InvokeWrapper>>
      client_tool_;
};

}  // namespace aimrt::plugins::ros2_plugin
