// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "ros2_plugin/ros2_adapter_rpc_server.h"
#include "aimrt_module_cpp_interface/rpc/rpc_status.h"
#include "aimrt_module_cpp_interface/util/buffer.h"
#include "aimrt_module_cpp_interface/util/type_support.h"
#include "core/rpc/rpc_backend_tools.h"
#include "ros2_plugin/global.h"

#include "ros2_plugin_proto/srv/ros_rpc_wrapper.hpp"

namespace aimrt::plugins::ros2_plugin {

Ros2AdapterServer::Ros2AdapterServer(
    const std::shared_ptr<rcl_node_t>& node_handle,
    const runtime::core::rpc::ServiceFuncWrapper& service_func_wrapper,
    const std::string& real_ros2_func_name,
    const rclcpp::QoS& qos)
    : rclcpp::ServiceBase(node_handle),
      service_func_wrapper_(service_func_wrapper),
      real_ros2_func_name_(real_ros2_func_name) {
  // rcl does the static memory allocation here
  service_handle_ = std::shared_ptr<rcl_service_t>(
      new rcl_service_t,
      [node_handle](rcl_service_t* service) {
        if (rcl_service_fini(service, node_handle.get()) != RCL_RET_OK) {
          RCLCPP_ERROR(
              rclcpp::get_node_logger(node_handle.get()).get_child("rclcpp"),
              "Error in destruction of rcl service handle: %s",
              rcl_get_error_string().str);
          rcl_reset_error();
        }
        delete service;
      });
  *service_handle_.get() = rcl_get_zero_initialized_service();

  rcl_service_options_t service_options = rcl_service_get_default_options();
  service_options.qos = qos.get_rmw_qos_profile();
  rcl_ret_t ret = rcl_service_init(
      service_handle_.get(),
      node_handle.get(),
      static_cast<const rosidl_service_type_support_t*>(service_func_wrapper.info.custom_type_support_ptr),
      real_ros2_func_name_.c_str(),
      &service_options);

  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      auto* rcl_node_handle = get_rcl_node_handle();
      // this will throw on any validation problem
      rcl_reset_error();
      rclcpp::expand_topic_or_service_name(
          real_ros2_func_name_,
          rcl_node_get_name(rcl_node_handle),
          rcl_node_get_namespace(rcl_node_handle),
          true);
    }

    AIMRT_WARN("Create ros2 service failed, func name '{}', err info: {}",
               service_func_wrapper_.info.func_name, rcl_get_error_string().str);
    rcl_reset_error();

  } else {
    AIMRT_TRACE("Create ros2 service successfully, func name '{}'",
                service_func_wrapper_.info.func_name);
  }
}

std::shared_ptr<void> Ros2AdapterServer::create_request() {
  AIMRT_TRACE("Create ros2 req, func name '{}'", service_func_wrapper_.info.func_name);
  return service_func_wrapper_.info.req_type_support_ref.CreateSharedPtr();
}

std::shared_ptr<rmw_request_id_t> Ros2AdapterServer::create_request_header() {
  AIMRT_TRACE("Create ros2 req header, func name '{}'", service_func_wrapper_.info.func_name);
  return std::make_shared<rmw_request_id_t>();
}

void Ros2AdapterServer::handle_request(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request) {
  if (!run_flag_.load()) return;

  AIMRT_TRACE("Handle ros2 req, func name '{}', seq num '{}'",
              service_func_wrapper_.info.func_name, request_header->sequence_number);

  // Create a service invoke wrapper
  auto service_invoke_wrapper_ptr = std::make_shared<runtime::core::rpc::InvokeWrapper>(
      runtime::core::rpc::InvokeWrapper{.info = service_func_wrapper_.info});

  // Create service ctx
  auto ctx_ptr = std::make_shared<aimrt::rpc::Context>(aimrt_rpc_context_type_t::AIMRT_RPC_SERVER_CONTEXT);
  service_invoke_wrapper_ptr->ctx_ref = ctx_ptr;

  ctx_ptr->SetFunctionName(service_func_wrapper_.info.func_name);
  ctx_ptr->SetMetaValue(AIMRT_RPC_CONTEXT_KEY_BACKEND, "ros2");

  // service req
  service_invoke_wrapper_ptr->req_ptr = request.get();

  // Create service rsp
  std::shared_ptr<void> service_rsp_ptr = service_func_wrapper_.info.rsp_type_support_ref.CreateSharedPtr();
  service_invoke_wrapper_ptr->rsp_ptr = service_rsp_ptr.get();

  // Set callback
  service_invoke_wrapper_ptr->callback =
      [this, service_rsp_ptr, ctx_ptr, request, request_header](aimrt::rpc::Status status) {
        AIMRT_TRACE("Handle ros2 req completed, func name '{}', seq num '{}'",
                    service_func_wrapper_.info.func_name, request_header->sequence_number);

        // Send rsp
        rcl_ret_t ret = rcl_send_response(service_handle_.get(), request_header.get(), service_rsp_ptr.get());

        if (ret != RCL_RET_OK) {
          AIMRT_WARN("Send ros2 rsp failed, func name '{}', err info: {}",
                     service_func_wrapper_.info.func_name, rcl_get_error_string().str);
          rcl_reset_error();
        }
      };

  service_func_wrapper_.service_func(service_invoke_wrapper_ptr);
}

Ros2AdapterWrapperServer::Ros2AdapterWrapperServer(
    const std::shared_ptr<rcl_node_t>& node_handle,
    const runtime::core::rpc::ServiceFuncWrapper& service_func_wrapper,
    const std::string& real_ros2_func_name,
    const rclcpp::QoS& qos)
    : rclcpp::ServiceBase(node_handle),
      service_func_wrapper_(service_func_wrapper),
      real_ros2_func_name_(real_ros2_func_name) {
  // rcl does the static memory allocation here
  service_handle_ = std::shared_ptr<rcl_service_t>(
      new rcl_service_t,
      [node_handle](rcl_service_t* service) {
        if (rcl_service_fini(service, node_handle.get()) != RCL_RET_OK) {
          RCLCPP_ERROR(
              rclcpp::get_node_logger(node_handle.get()).get_child("rclcpp"),
              "Error in destruction of rcl service handle: %s",
              rcl_get_error_string().str);
          rcl_reset_error();
        }
        delete service;
      });
  *service_handle_.get() = rcl_get_zero_initialized_service();

  rcl_service_options_t service_options = rcl_service_get_default_options();
  service_options.qos = qos.get_rmw_qos_profile();
  rcl_ret_t ret = rcl_service_init(
      service_handle_.get(),
      node_handle.get(),
      rosidl_typesupport_cpp::get_service_type_support_handle<ros2_plugin_proto::srv::RosRpcWrapper>(),
      real_ros2_func_name_.c_str(),
      &service_options);

  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      auto* rcl_node_handle = get_rcl_node_handle();
      // this will throw on any validation problem
      rcl_reset_error();
      rclcpp::expand_topic_or_service_name(
          real_ros2_func_name_,
          rcl_node_get_name(rcl_node_handle),
          rcl_node_get_namespace(rcl_node_handle),
          true);
    }

    AIMRT_WARN("Create ros2 service failed, func name '{}', err info: {}",
               service_func_wrapper_.info.func_name, rcl_get_error_string().str);
    rcl_reset_error();

  } else {
    AIMRT_TRACE("Create ros2 service successfully, func name '{}'",
                service_func_wrapper_.info.func_name);
  }
}

std::shared_ptr<void> Ros2AdapterWrapperServer::create_request() {
  AIMRT_TRACE("Create ros2 req, func name '{}'", service_func_wrapper_.info.func_name);
  return std::make_shared<ros2_plugin_proto::srv::RosRpcWrapper::Request>();
}

std::shared_ptr<rmw_request_id_t> Ros2AdapterWrapperServer::create_request_header() {
  AIMRT_TRACE("Create ros2 req header, func name '{}'", service_func_wrapper_.info.func_name);
  return std::make_shared<rmw_request_id_t>();
}

void Ros2AdapterWrapperServer::handle_request(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request) {
  if (!run_flag_.load()) return;

  AIMRT_TRACE("Handle ros2 req, func name '{}', seq num '{}'",
              service_func_wrapper_.info.func_name, request_header->sequence_number);

  // Create a service invoke wrapper
  auto service_invoke_wrapper_ptr = std::make_shared<runtime::core::rpc::InvokeWrapper>(
      runtime::core::rpc::InvokeWrapper{.info = service_func_wrapper_.info});
  const auto& info = service_invoke_wrapper_ptr->info;

  // Create ctx
  auto ctx_ptr = std::make_shared<aimrt::rpc::Context>(aimrt_rpc_context_type_t::AIMRT_RPC_SERVER_CONTEXT);
  service_invoke_wrapper_ptr->ctx_ref = ctx_ptr;

  // Get fields
  auto& wrapper_req = *(static_cast<ros2_plugin_proto::srv::RosRpcWrapper::Request*>(request.get()));

  for (size_t ii = 0; ii + 1 < wrapper_req.context.size(); ii += 2) {
    const auto& key = wrapper_req.context[ii];
    const auto& val = wrapper_req.context[ii + 1];
    if (ii == 0 && key == "aimrt-timeout") {
      ctx_ptr->SetTimeout(std::chrono::nanoseconds(std::stoll(val)));
      continue;
    }
    ctx_ptr->SetMetaValue(key, val);
  }

  ctx_ptr->SetMetaValue(AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE, wrapper_req.serialization_type);

  ctx_ptr->SetFunctionName(service_func_wrapper_.info.func_name);
  ctx_ptr->SetMetaValue(AIMRT_RPC_CONTEXT_KEY_BACKEND, "ros2");

  // service req deserialization
  aimrt_buffer_view_t buffer_view{
      .data = wrapper_req.data.data(),
      .len = wrapper_req.data.size()};

  aimrt_buffer_array_view_t buffer_array_view{
      .data = &buffer_view,
      .len = 1};

  std::shared_ptr<void> service_req_ptr = info.req_type_support_ref.CreateSharedPtr();
  service_invoke_wrapper_ptr->req_ptr = service_req_ptr.get();

  bool deserialize_ret = info.req_type_support_ref.Deserialize(
      wrapper_req.serialization_type, buffer_array_view, service_req_ptr.get());

  if (!deserialize_ret) [[unlikely]] {
    AIMRT_ERROR("ROS2 wrapper req deserialize failed.");

    ReturnRspWithStatusCode(request_header, AIMRT_RPC_STATUS_SVR_DESERIALIZATION_FAILED);

    return;
  }

  // Create service rsp
  std::shared_ptr<void> service_rsp_ptr = info.rsp_type_support_ref.CreateSharedPtr();
  service_invoke_wrapper_ptr->rsp_ptr = service_rsp_ptr.get();

  // Set callback
  service_invoke_wrapper_ptr->callback =
      [this,
       service_invoke_wrapper_ptr,
       ctx_ptr,
       service_req_ptr,
       service_rsp_ptr,
       serialization_type{std::move(wrapper_req.serialization_type)},
       request_header](aimrt::rpc::Status status) {
        AIMRT_TRACE("Handle ros2 req completed, func name '{}', seq num '{}'",
                    service_func_wrapper_.info.func_name, request_header->sequence_number);

        if (!status.OK()) [[unlikely]] {
          // If the code is not suc, then deserialization is not necessary
          ReturnRspWithStatusCode(request_header, status.Code());
          return;
        }

        // service rsp serialization
        auto buffer_array_view_ptr = aimrt::runtime::core::rpc::TrySerializeRspWithCache(*service_invoke_wrapper_ptr, serialization_type);
        if (!buffer_array_view_ptr) [[unlikely]] {
          ReturnRspWithStatusCode(request_header, AIMRT_RPC_STATUS_SVR_SERIALIZATION_FAILED);
          return;
        }

        const auto* buffer_array_data = buffer_array_view_ptr->Data();
        const size_t buffer_array_len = buffer_array_view_ptr->Size();
        size_t rsp_size = buffer_array_view_ptr->BufferSize();

        auto wrapper_rsp_ptr = std::make_shared<ros2_plugin_proto::srv::RosRpcWrapper::Response>();
        wrapper_rsp_ptr->code = 0;
        wrapper_rsp_ptr->serialization_type = serialization_type;
        wrapper_rsp_ptr->data.resize(rsp_size);

        auto* cur_pos = wrapper_rsp_ptr->data.data();
        for (size_t ii = 0; ii < buffer_array_len; ++ii) {
          memcpy(cur_pos, buffer_array_data[ii].data, buffer_array_data[ii].len);
          cur_pos += buffer_array_data[ii].len;
        }

        // Send rsp
        rcl_ret_t ret = rcl_send_response(service_handle_.get(), request_header.get(), wrapper_rsp_ptr.get());

        if (ret != RCL_RET_OK) {
          AIMRT_WARN("Send ros2 rsp failed, func name '{}', err info: {}",
                     service_func_wrapper_.info.func_name, rcl_get_error_string().str);
          rcl_reset_error();
        }
      };

  // Call service rpc
  service_func_wrapper_.service_func(service_invoke_wrapper_ptr);
}

void Ros2AdapterWrapperServer::ReturnRspWithStatusCode(
    const std::shared_ptr<rmw_request_id_t>& request_header, uint32_t code) {
  auto wrapper_rsp_ptr = std::make_shared<ros2_plugin_proto::srv::RosRpcWrapper::Response>();
  wrapper_rsp_ptr->code = code;

  // Send rsp
  rcl_ret_t ret = rcl_send_response(service_handle_.get(), request_header.get(), wrapper_rsp_ptr.get());

  if (ret != RCL_RET_OK) {
    AIMRT_WARN("Send ros2 rsp failed, func name '{}', err info: {}",
               service_func_wrapper_.info.func_name, rcl_get_error_string().str);
    rcl_reset_error();
  }
}

}  // namespace aimrt::plugins::ros2_plugin