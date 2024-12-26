// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "ros2_plugin/ros2_adapter_rpc_client.h"

#include <utility>
#include "aimrt_module_cpp_interface/rpc/rpc_status.h"
#include "aimrt_module_cpp_interface/util/buffer.h"
#include "aimrt_module_cpp_interface/util/type_support.h"
#include "core/rpc/rpc_backend_tools.h"
#include "ros2_plugin/global.h"

#include "ros2_plugin_proto/srv/ros_rpc_wrapper.hpp"

namespace aimrt::plugins::ros2_plugin {

Ros2AdapterClient::Ros2AdapterClient(
    rclcpp::node_interfaces::NodeBaseInterface* node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    const runtime::core::rpc::ClientFuncWrapper& client_func_wrapper,
    const std::string& real_ros2_func_name,
    const rclcpp::QoS& qos,
    aimrt::executor::ExecutorRef timeout_executor)
    : rclcpp::ClientBase(node_base, std::move(node_graph)),
      client_func_wrapper_(client_func_wrapper),
      real_ros2_func_name_(real_ros2_func_name) {
  rcl_client_options_t client_options = rcl_client_get_default_options();
  client_options.qos = qos.get_rmw_qos_profile();

  rcl_ret_t ret = rcl_client_init(
      this->get_client_handle().get(),
      this->get_rcl_node_handle(),
      static_cast<const rosidl_service_type_support_t*>(client_func_wrapper_.info.custom_type_support_ptr),
      real_ros2_func_name_.c_str(),
      &client_options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      auto* rcl_node_handle = this->get_rcl_node_handle();
      // this will throw on any validation problem
      rcl_reset_error();
      rclcpp::expand_topic_or_service_name(
          real_ros2_func_name_,
          rcl_node_get_name(rcl_node_handle),
          rcl_node_get_namespace(rcl_node_handle),
          true);
    }

    AIMRT_WARN("Create ros2 client failed, func name '{}', err info: {}",
               client_func_wrapper_.info.func_name, rcl_get_error_string().str);
    rcl_reset_error();
  } else {
    AIMRT_TRACE("Create ros2 client successfully, func name '{}'",
                client_func_wrapper_.info.func_name);
  }

  if (timeout_executor) {
    client_tool_.RegisterTimeoutExecutor(timeout_executor);
    client_tool_.RegisterTimeoutHandle(
        [](auto&& client_invoke_wrapper_ptr) {
          client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_TIMEOUT));
        });
  }
}

std::shared_ptr<void> Ros2AdapterClient::create_response() {
  AIMRT_TRACE("Create ros2 rsp, func name '{}'", client_func_wrapper_.info.func_name);
  return client_func_wrapper_.info.rsp_type_support_ref.CreateSharedPtr();
}

std::shared_ptr<rmw_request_id_t> Ros2AdapterClient::create_request_header() {
  AIMRT_TRACE("Create ros2 req header, func name '{}'", client_func_wrapper_.info.func_name);
  return std::make_shared<rmw_request_id_t>();
}

void Ros2AdapterClient::handle_response(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response) {
  if (!run_flag_.load()) return;

  AIMRT_TRACE("Handle ros2 rsp, func name '{}', seq num '{}'",
              client_func_wrapper_.info.func_name, request_header->sequence_number);

  auto msg_recorder = client_tool_.GetRecord(request_header->sequence_number);
  if (!msg_recorder) [[unlikely]] {
    // 未找到记录，说明此次调用已经超时了，走了超时处理后删掉了记录
    AIMRT_TRACE("Can not get req id {} from recorder.", request_header->sequence_number);
    return;
  }

  auto client_invoke_wrapper_ptr = std::move(*msg_recorder);
  client_func_wrapper_.info.rsp_type_support_ref.Move(response.get(), client_invoke_wrapper_ptr->rsp_ptr);
  client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_OK));
}

void Ros2AdapterClient::Invoke(
    const std::shared_ptr<runtime::core::rpc::InvokeWrapper>& client_invoke_wrapper_ptr) {
  AIMRT_TRACE("Invoke ros2 req, func name '{}'",
              client_invoke_wrapper_ptr->info.func_name);

  auto timeout = client_invoke_wrapper_ptr->ctx_ref.Timeout();
  auto record_ptr = client_invoke_wrapper_ptr;

  int64_t sequence_number = 0;
  rcl_ret_t ret = rcl_send_request(
      get_client_handle().get(), client_invoke_wrapper_ptr->req_ptr, &sequence_number);

  if (RCL_RET_OK != ret) [[unlikely]] {
    AIMRT_WARN("Ros2 client send req failed, func name '{}', err info: {}",
               client_invoke_wrapper_ptr->info.func_name, rcl_get_error_string().str);
    rcl_reset_error();
    client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_SEND_REQ_FAILED));
    return;
  }

  bool record_ret = client_tool_.Record(sequence_number, timeout, std::move(record_ptr));

  if (!record_ret) [[unlikely]] {
    AIMRT_ERROR("Failed to record msg.");
    client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
    return;
  }
}

Ros2AdapterWrapperClient::Ros2AdapterWrapperClient(
    rclcpp::node_interfaces::NodeBaseInterface* node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    const runtime::core::rpc::ClientFuncWrapper& client_func_wrapper,
    const std::string& real_ros2_func_name,
    const rclcpp::QoS& qos,
    aimrt::executor::ExecutorRef timeout_executor)
    : rclcpp::ClientBase(node_base, std::move(node_graph)),
      client_func_wrapper_(client_func_wrapper),
      real_ros2_func_name_(real_ros2_func_name) {
  rcl_client_options_t client_options = rcl_client_get_default_options();
  client_options.qos = qos.get_rmw_qos_profile();

  rcl_ret_t ret = rcl_client_init(
      this->get_client_handle().get(),
      this->get_rcl_node_handle(),
      rosidl_typesupport_cpp::get_service_type_support_handle<ros2_plugin_proto::srv::RosRpcWrapper>(),
      real_ros2_func_name_.c_str(),
      &client_options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      auto* rcl_node_handle = this->get_rcl_node_handle();
      // this will throw on any validation problem
      rcl_reset_error();
      rclcpp::expand_topic_or_service_name(
          real_ros2_func_name_,
          rcl_node_get_name(rcl_node_handle),
          rcl_node_get_namespace(rcl_node_handle),
          true);
    }

    AIMRT_WARN("Create ros2 client failed, func name '{}', err info: {}",
               client_func_wrapper_.info.func_name, rcl_get_error_string().str);
    rcl_reset_error();
  } else {
    AIMRT_TRACE("Create ros2 client successfully, func name '{}'",
                client_func_wrapper_.info.func_name);
  }

  if (timeout_executor) {
    client_tool_.RegisterTimeoutExecutor(timeout_executor);
    client_tool_.RegisterTimeoutHandle(
        [](auto&& client_invoke_wrapper_ptr) {
          client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_TIMEOUT));
        });
  }
}

std::shared_ptr<void> Ros2AdapterWrapperClient::create_response() {
  AIMRT_TRACE("Create ros2 rsp, func name '{}'", client_func_wrapper_.info.func_name);
  return std::make_shared<ros2_plugin_proto::srv::RosRpcWrapper::Response>();
}

std::shared_ptr<rmw_request_id_t> Ros2AdapterWrapperClient::create_request_header() {
  AIMRT_TRACE("Create ros2 req header, func name '{}'", client_func_wrapper_.info.func_name);
  return std::make_shared<rmw_request_id_t>();
}

void Ros2AdapterWrapperClient::handle_response(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response) {
  if (!run_flag_.load()) return;

  AIMRT_TRACE("Handle ros2 rsp, func name '{}', seq num '{}'",
              client_func_wrapper_.info.func_name, request_header->sequence_number);

  auto msg_recorder = client_tool_.GetRecord(request_header->sequence_number);
  if (!msg_recorder) [[unlikely]] {
    // 未找到记录，说明此次调用已经超时了，走了超时处理后删掉了记录
    AIMRT_TRACE("Can not get req id {} from recorder.", request_header->sequence_number);
    return;
  }

  auto client_invoke_wrapper_ptr = std::move(*msg_recorder);

  // client rsp 创建、反序列化
  auto& wrapper_rsp = *(static_cast<ros2_plugin_proto::srv::RosRpcWrapper::Response*>(response.get()));

  if (wrapper_rsp.code) {
    client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(wrapper_rsp.code));
    return;
  }

  aimrt_buffer_view_t buffer_view{
      .data = wrapper_rsp.data.data(),
      .len = wrapper_rsp.data.size()};

  aimrt_buffer_array_view_t buffer_array_view{
      .data = &buffer_view,
      .len = 1};

  bool deserialize_ret = client_func_wrapper_.info.rsp_type_support_ref.Deserialize(
      wrapper_rsp.serialization_type, buffer_array_view, client_invoke_wrapper_ptr->rsp_ptr);

  if (!deserialize_ret) {
    // 调用回调
    client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_DESERIALIZATION_FAILED));
    return;
  }

  client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_OK));
}

void Ros2AdapterWrapperClient::Invoke(
    const std::shared_ptr<runtime::core::rpc::InvokeWrapper>& client_invoke_wrapper_ptr) {
  AIMRT_TRACE("Invoke ros2 req, func name '{}'", client_invoke_wrapper_ptr->info.func_name);

  // 序列化 client req
  auto serialization_type =
      client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE);

  auto buffer_array_view_ptr = aimrt::runtime::core::rpc::TrySerializeReqWithCache(*client_invoke_wrapper_ptr, serialization_type);
  if (!buffer_array_view_ptr) [[unlikely]] {
    client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_SERIALIZATION_FAILED));
    return;
  }

  // 填wrapper_req
  const auto* buffer_array_data = buffer_array_view_ptr->Data();
  const size_t buffer_array_len = buffer_array_view_ptr->Size();
  size_t req_size = buffer_array_view_ptr->BufferSize();

  ros2_plugin_proto::srv::RosRpcWrapper::Request wrapper_req;
  wrapper_req.serialization_type = serialization_type;

  const auto& keys = client_invoke_wrapper_ptr->ctx_ref.GetMetaKeys();
  wrapper_req.context.reserve(2 * (keys.size() + 1));
  wrapper_req.context.emplace_back("aimrt-timeout");
  wrapper_req.context.emplace_back(std::to_string(client_invoke_wrapper_ptr->ctx_ref.Timeout().count()));
  for (const auto& key : keys) {
    wrapper_req.context.emplace_back(key);
    wrapper_req.context.emplace_back(client_invoke_wrapper_ptr->ctx_ref.GetMetaValue(key));
  }

  wrapper_req.data.resize(req_size);

  auto* cur_pos = wrapper_req.data.data();
  for (size_t ii = 0; ii < buffer_array_len; ++ii) {
    memcpy(cur_pos, buffer_array_data[ii].data, buffer_array_data[ii].len);
    cur_pos += buffer_array_data[ii].len;
  }

  auto timeout = client_invoke_wrapper_ptr->ctx_ref.Timeout();
  auto record_ptr = client_invoke_wrapper_ptr;

  int64_t sequence_number = 0;
  rcl_ret_t ret = rcl_send_request(
      get_client_handle().get(), &wrapper_req, &sequence_number);

  if (RCL_RET_OK != ret) [[unlikely]] {
    AIMRT_WARN("Ros2 client send req failed, func name '{}', err info: {}",
               client_invoke_wrapper_ptr->info.func_name, rcl_get_error_string().str);
    rcl_reset_error();
    client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_SEND_REQ_FAILED));
    return;
  }

  bool record_ret = client_tool_.Record(sequence_number, timeout, std::move(record_ptr));

  if (!record_ret) [[unlikely]] {
    AIMRT_ERROR("Failed to record msg.");
    client_invoke_wrapper_ptr->callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR));
    return;
  }
}

}  // namespace aimrt::plugins::ros2_plugin