#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import os
import sys


def get_snake_case_name(text):
    lst = []
    for index, char in enumerate(text):
        if char.isupper() and index != 0:
            lst.append("_")
        lst.append(char)

    return "".join(lst).lower()


def gen_h_file(pkg_name, srv_filename):
    t_h_file = '''/**
 * @file {{srv_filename}}.aimrt_rpc.srv.h
 * @brief This file was generated by ros2-aimrt_rpc-gen-code-tool, do not edit it!!!
 */
#pragma once

#include <future>

#include "aimrt_module_cpp_interface/rpc/rpc_handle.h"
#include "aimrt_module_cpp_interface/rpc/rpc_status.h"
#include "aimrt_module_cpp_interface/util/version.h"
#include "aimrt_module_cpp_interface/co/task.h"

#include "{{pkg_name}}/srv/{{snake_case_srv_filename}}.hpp"

static_assert(10000 <= AIMRT_RUNTIME_VERSION_INT,
              "AIMRT_RUNTIME_VERSION is older than generated code version 0.10.0");
static_assert(AIMRT_MIN_GENCODE_VERSION_INT <= 10000,
              "AIMRT_MIN_GENCODE_VERSION is greater than generated code version 0.10.0");


namespace {{pkg_name}} {
namespace srv {

class {{srv_filename}}SyncService : public aimrt::rpc::ServiceBase {
 public:
  {{srv_filename}}SyncService();
  ~{{srv_filename}}SyncService() override = default;

  virtual aimrt::rpc::Status {{srv_filename}}(
      aimrt::rpc::ContextRef ctx_ref,
      const {{srv_filename}}_Request& req,
      {{srv_filename}}_Response& rsp) {
    return aimrt::rpc::Status(AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED);
  }
};

class {{srv_filename}}AsyncService : public aimrt::rpc::ServiceBase {
 public:
  {{srv_filename}}AsyncService();
  ~{{srv_filename}}AsyncService() override = default;

  virtual void {{srv_filename}}(
      aimrt::rpc::ContextRef ctx_ref,
      const {{srv_filename}}_Request& req,
      {{srv_filename}}_Response& rsp,
      std::function<void(aimrt::rpc::Status)>&& callback) {
    callback(aimrt::rpc::Status(AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED));
  }
};

class {{srv_filename}}CoService : public aimrt::rpc::CoServiceBase {
 public:
  {{srv_filename}}CoService();
  ~{{srv_filename}}CoService() override = default;

  virtual aimrt::co::Task<aimrt::rpc::Status> {{srv_filename}}(
      aimrt::rpc::ContextRef ctx_ref,
      const {{srv_filename}}_Request& req,
      {{srv_filename}}_Response& rsp) {
    co_return aimrt::rpc::Status(AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED);
  }
};

bool Register{{srv_filename}}ClientFunc(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name);
bool Register{{srv_filename}}ClientFunc(aimrt::rpc::RpcHandleRef rpc_handle_ref);

class {{srv_filename}}SyncProxy : public aimrt::rpc::SyncProxyBase {
 public:
  explicit {{srv_filename}}SyncProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref);
  explicit {{srv_filename}}SyncProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name);
  ~{{srv_filename}}SyncProxy() = default;

  static bool RegisterClientFunc(aimrt::rpc::RpcHandleRef rpc_handle_ref) {
    return Register{{srv_filename}}ClientFunc(rpc_handle_ref);
  }

  static bool RegisterClientFunc(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name) {
    return Register{{srv_filename}}ClientFunc(rpc_handle_ref, service_name);
  }

  aimrt::rpc::Status {{srv_filename}}(
      aimrt::rpc::ContextRef ctx_ref,
      const {{srv_filename}}_Request& req,
      {{srv_filename}}_Response& rsp);

  aimrt::rpc::Status {{srv_filename}}(
      const {{srv_filename}}_Request& req,
      {{srv_filename}}_Response& rsp) {
    return {{srv_filename}}(aimrt::rpc::ContextRef(), req, rsp);
  }
};

class {{srv_filename}}AsyncProxy : public aimrt::rpc::AsyncProxyBase {
 public:
  explicit {{srv_filename}}AsyncProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref);
  explicit {{srv_filename}}AsyncProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name);
  ~{{srv_filename}}AsyncProxy() = default;

  static bool RegisterClientFunc(aimrt::rpc::RpcHandleRef rpc_handle_ref) {
    return Register{{srv_filename}}ClientFunc(rpc_handle_ref);
  }

  static bool RegisterClientFunc(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name) {
    return Register{{srv_filename}}ClientFunc(rpc_handle_ref, service_name);
  }

  void {{srv_filename}}(
      aimrt::rpc::ContextRef ctx_ref,
      const {{srv_filename}}_Request& req,
      {{srv_filename}}_Response& rsp,
      std::function<void(aimrt::rpc::Status)>&& callback);

  void {{srv_filename}}(
      const {{srv_filename}}_Request& req,
      {{srv_filename}}_Response& rsp,
      std::function<void(aimrt::rpc::Status)>&& callback) {
    {{srv_filename}}(aimrt::rpc::ContextRef(), req, rsp, std::move(callback));
  }
};

class {{srv_filename}}FutureProxy : public aimrt::rpc::FutureProxyBase {
 public:
  explicit {{srv_filename}}FutureProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref);
  explicit {{srv_filename}}FutureProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name);
  ~{{srv_filename}}FutureProxy() = default;

  static bool RegisterClientFunc(aimrt::rpc::RpcHandleRef rpc_handle_ref) {
    return Register{{srv_filename}}ClientFunc(rpc_handle_ref);
  }

  static bool RegisterClientFunc(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name) {
    return Register{{srv_filename}}ClientFunc(rpc_handle_ref, service_name);
  }

  std::future<aimrt::rpc::Status> {{srv_filename}}(
      aimrt::rpc::ContextRef ctx_ref,
      const {{srv_filename}}_Request& req,
      {{srv_filename}}_Response& rsp);

  std::future<aimrt::rpc::Status> {{srv_filename}}(
      const {{srv_filename}}_Request& req,
      {{srv_filename}}_Response& rsp) {
    return {{srv_filename}}(aimrt::rpc::ContextRef(), req, rsp);
  }
};

class {{srv_filename}}CoProxy : public aimrt::rpc::CoProxyBase {
 public:
  explicit {{srv_filename}}CoProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref);
  explicit {{srv_filename}}CoProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name);
  ~{{srv_filename}}CoProxy() = default;

  static bool RegisterClientFunc(aimrt::rpc::RpcHandleRef rpc_handle_ref) {
    return Register{{srv_filename}}ClientFunc(rpc_handle_ref);
  }

  static bool RegisterClientFunc(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name) {
    return Register{{srv_filename}}ClientFunc(rpc_handle_ref, service_name);
  }

  aimrt::co::Task<aimrt::rpc::Status> {{srv_filename}}(
      aimrt::rpc::ContextRef ctx_ref,
      const {{srv_filename}}_Request& req,
      {{srv_filename}}_Response& rsp);

  aimrt::co::Task<aimrt::rpc::Status> {{srv_filename}}(
      const {{srv_filename}}_Request& req,
      {{srv_filename}}_Response& rsp) {
    return {{srv_filename}}(aimrt::rpc::ContextRef(), req, rsp);
  }
};

}  // namespace srv
}  // namespace {{pkg_name}}
'''

    h_file = str = t_h_file \
        .replace("{{srv_filename}}", srv_filename) \
        .replace("{{snake_case_srv_filename}}", get_snake_case_name(srv_filename)) \
        .replace("{{pkg_name}}", pkg_name)

    return h_file


def gen_cc_file(pkg_name, srv_filename):
    t_cc_file = '''/**
 * @file {{srv_filename}}.aimrt_rpc.srv.cc
 * @brief This file was generated by ros2-aimrt_rpc-gen-code-tool, do not edit it!!!
 */

#include "{{srv_filename}}.aimrt_rpc.srv.h"

#include "aimrt_module_cpp_interface/co/inline_scheduler.h"
#include "aimrt_module_cpp_interface/co/on.h"
#include "aimrt_module_cpp_interface/co/start_detached.h"
#include "aimrt_module_cpp_interface/co/then.h"
#include "aimrt_module_ros2_interface/util/ros2_type_support.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

namespace {{pkg_name}} {
namespace srv {

static constexpr std::string_view kRpcType = "ros2";
static constexpr std::string_view kServiceName = "{{pkg_name}}/srv";
static constexpr std::string_view kFuncName = "{{srv_filename}}";

{{srv_filename}}SyncService::{{srv_filename}}SyncService() : aimrt::rpc::ServiceBase(kRpcType, kServiceName) {
  aimrt::rpc::ServiceFunc service_callback(
      [this](const aimrt_rpc_context_base_t* ctx, const void* req, void* rsp, aimrt_function_base_t* result_callback_ptr) {
        aimrt::rpc::ServiceCallback result_callback(result_callback_ptr);

        aimrt::rpc::ContextRef ctx_ref(ctx);

        auto status = {{srv_filename}}(
            ctx_ref,
            *static_cast<const {{srv_filename}}_Request*>(req),
            *static_cast<{{srv_filename}}_Response*>(rsp));

        result_callback(status.Code());
      });
  RegisterServiceFunc(
      kFuncName,
      rosidl_typesupport_cpp::get_service_type_support_handle<{{pkg_name}}::srv::{{srv_filename}}>(),
      aimrt::GetRos2MessageTypeSupport<{{srv_filename}}_Request>(),
      aimrt::GetRos2MessageTypeSupport<{{srv_filename}}_Response>(),
      std::move(service_callback));
}

{{srv_filename}}AsyncService::{{srv_filename}}AsyncService() : aimrt::rpc::ServiceBase(kRpcType, kServiceName) {
  aimrt::rpc::ServiceFunc service_callback(
      [this](const aimrt_rpc_context_base_t* ctx, const void* req, void* rsp, aimrt_function_base_t* result_callback_ptr) {
        auto result_callback_func_ptr = std::make_shared<aimrt::rpc::ServiceCallback>(result_callback_ptr);

        aimrt::rpc::ContextRef ctx_ref(ctx);

        {{srv_filename}}(
            ctx_ref,
            *static_cast<const {{srv_filename}}_Request*>(req),
            *static_cast<{{srv_filename}}_Response*>(rsp),
            [result_callback_func_ptr{std::move(result_callback_func_ptr)}](aimrt::rpc::Status status) {
              (*result_callback_func_ptr)(status.Code());
            });
      });
  RegisterServiceFunc(
      kFuncName,
      rosidl_typesupport_cpp::get_service_type_support_handle<{{pkg_name}}::srv::{{srv_filename}}>(),
      aimrt::GetRos2MessageTypeSupport<{{srv_filename}}_Request>(),
      aimrt::GetRos2MessageTypeSupport<{{srv_filename}}_Response>(),
      std::move(service_callback));
}

{{srv_filename}}CoService::{{srv_filename}}CoService() : aimrt::rpc::CoServiceBase(kRpcType, kServiceName) {
  aimrt::rpc::ServiceFunc service_callback(
      [this](const aimrt_rpc_context_base_t* ctx, const void* req, void* rsp, aimrt_function_base_t* result_callback_ptr) {
        auto handle_ptr = std::make_unique<const aimrt::rpc::CoRpcHandle>(
            [this](aimrt::rpc::ContextRef ctx_ref, const void* req_ptr, void* rsp_ptr)
                -> aimrt::co::Task<aimrt::rpc::Status> {
              return {{srv_filename}}(
                  ctx_ref,
                  *static_cast<const {{srv_filename}}_Request*>(req_ptr),
                  *static_cast<{{srv_filename}}_Response*>(rsp_ptr));
            });

        aimrt::rpc::ServiceCallback result_callback(result_callback_ptr);

        aimrt::rpc::ContextRef ctx_ref(ctx);

        auto* ptr = handle_ptr.get();
        aimrt::co::StartDetached(
            aimrt::co::On(
                aimrt::co::InlineScheduler(),
                filter_mgr_.InvokeRpc(*ptr, ctx_ref, req, rsp)) |
            aimrt::co::Then(
                [handle_ptr{std::move(handle_ptr)}, result_callback{std::move(result_callback)}](aimrt::rpc::Status status) {
                  result_callback(status.Code());
                }));
      });
  RegisterServiceFunc(
      kFuncName,
      rosidl_typesupport_cpp::get_service_type_support_handle<{{pkg_name}}::srv::{{srv_filename}}>(),
      aimrt::GetRos2MessageTypeSupport<{{srv_filename}}_Request>(),
      aimrt::GetRos2MessageTypeSupport<{{srv_filename}}_Response>(),
      std::move(service_callback));
}

bool Register{{srv_filename}}ClientFunc(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name) {
  return rpc_handle_ref.RegisterClientFunc(
      kRpcType,
      service_name,
      kFuncName,
      rosidl_typesupport_cpp::get_service_type_support_handle<{{pkg_name}}::srv::{{srv_filename}}>(),
      aimrt::GetRos2MessageTypeSupport<{{srv_filename}}_Request>(),
      aimrt::GetRos2MessageTypeSupport<{{srv_filename}}_Response>());
}

bool Register{{srv_filename}}ClientFunc(aimrt::rpc::RpcHandleRef rpc_handle_ref) {
  return Register{{srv_filename}}ClientFunc(rpc_handle_ref, kServiceName);
}

{{srv_filename}}SyncProxy::{{srv_filename}}SyncProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref)
    : aimrt::rpc::SyncProxyBase(rpc_handle_ref, kRpcType, kServiceName) {}

{{srv_filename}}SyncProxy::{{srv_filename}}SyncProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name)
    : aimrt::rpc::SyncProxyBase(rpc_handle_ref, kRpcType, service_name) {}

aimrt::rpc::Status {{srv_filename}}SyncProxy::{{srv_filename}}(
    aimrt::rpc::ContextRef ctx_ref,
    const {{srv_filename}}_Request& req,
    {{srv_filename}}_Response& rsp) {
  const std::string& full_func_name = aimrt::rpc::GetFullFuncName(rpc_type_, service_name_, kFuncName);
  return Invoke(full_func_name, ctx_ref, req, rsp);
}

{{srv_filename}}AsyncProxy::{{srv_filename}}AsyncProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref)
    : aimrt::rpc::AsyncProxyBase(rpc_handle_ref, kRpcType, kServiceName) {}

{{srv_filename}}AsyncProxy::{{srv_filename}}AsyncProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name)
    : aimrt::rpc::AsyncProxyBase(rpc_handle_ref, kRpcType, service_name) {}

void {{srv_filename}}AsyncProxy::{{srv_filename}}(
    aimrt::rpc::ContextRef ctx_ref,
    const {{srv_filename}}_Request& req,
    {{srv_filename}}_Response& rsp,
    std::function<void(aimrt::rpc::Status)>&& callback) {
  const std::string& full_func_name = aimrt::rpc::GetFullFuncName(rpc_type_, service_name_, kFuncName);
  Invoke(full_func_name, ctx_ref, req, rsp, std::move(callback));
}

{{srv_filename}}FutureProxy::{{srv_filename}}FutureProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref)
    : aimrt::rpc::FutureProxyBase(rpc_handle_ref, kRpcType, kServiceName) {}

{{srv_filename}}FutureProxy::{{srv_filename}}FutureProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name)
    : aimrt::rpc::FutureProxyBase(rpc_handle_ref, kRpcType, service_name) {}

std::future<aimrt::rpc::Status> {{srv_filename}}FutureProxy::{{srv_filename}}(
    aimrt::rpc::ContextRef ctx_ref,
    const {{srv_filename}}_Request& req,
    {{srv_filename}}_Response& rsp) {
  const std::string& full_func_name = aimrt::rpc::GetFullFuncName(rpc_type_, service_name_, kFuncName);
  return Invoke(full_func_name, ctx_ref, req, rsp);
}

{{srv_filename}}CoProxy::{{srv_filename}}CoProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref)
    : aimrt::rpc::CoProxyBase(rpc_handle_ref, kRpcType, kServiceName) {}

{{srv_filename}}CoProxy::{{srv_filename}}CoProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name)
    : aimrt::rpc::CoProxyBase(rpc_handle_ref, kRpcType, service_name) {}

aimrt::co::Task<aimrt::rpc::Status> {{srv_filename}}CoProxy::{{srv_filename}}(
    aimrt::rpc::ContextRef ctx_ref,
    const {{srv_filename}}_Request& req,
    {{srv_filename}}_Response& rsp) {
  const std::string& full_func_name = aimrt::rpc::GetFullFuncName(rpc_type_, service_name_, kFuncName);
  co_return co_await Invoke(full_func_name, ctx_ref, req, rsp);
}

}  // namespace srv
}  // namespace {{pkg_name}}
'''

    cc_file = str = t_cc_file \
        .replace("{{srv_filename}}", srv_filename) \
        .replace("{{pkg_name}}", pkg_name)
    return cc_file


if __name__ == '__main__':
    pkg_name = ""
    srv_file = ""
    output_path = ""
    for arg in sys.argv:
        kv = arg.split('=')
        if (kv[0] == '--pkg_name'):
            pkg_name = kv[1]
        elif (kv[0] == '--srv_file'):
            srv_file = kv[1]
        elif (kv[0] == '--output_path'):
            output_path = kv[1]

    (path, file) = os.path.split(srv_file)
    (filename, ext) = os.path.splitext(file)
    cc_file_path = os.path.join(output_path, filename + ".aimrt_rpc.srv.cc")
    h_file_path = os.path.join(output_path, filename + ".aimrt_rpc.srv.h")

    # cc file
    f_cc_file = open(cc_file_path, 'w')
    f_cc_file.write(gen_cc_file(pkg_name, filename))
    f_cc_file.close()

    # h file
    f_h_file = open(h_file_path, 'w')
    f_h_file.write(gen_h_file(pkg_name, filename))
    f_h_file.close()
