// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "pybind11/pybind11.h"

#include <future>
#include <string>
#include <utility>

#include "aimrt_module_cpp_interface/rpc/rpc_handle.h"
#include "python_runtime/export_pb_type_support.h"
#include "rpc/rpc_context_base.h"

#ifdef AIMRT_BUILD_WITH_ROS2
  #include "python_runtime/export_ros2_type_support.h"
  #include "python_runtime/ros2_type_support_utils.h"
#endif

namespace aimrt::runtime::python_runtime {

inline void ExportRpcStatus(const pybind11::object& m) {
  using aimrt::rpc::Status;

  pybind11::enum_<aimrt_rpc_status_code_t>(m, "RpcStatusRetCode")
      .value("OK", AIMRT_RPC_STATUS_OK)
      .value("UNKNOWN", AIMRT_RPC_STATUS_UNKNOWN)
      .value("TIMEOUT", AIMRT_RPC_STATUS_TIMEOUT)
      // svr side
      .value("SVR_UNKNOWN", AIMRT_RPC_STATUS_SVR_UNKNOWN)
      .value("SVR_BACKEND_INTERNAL_ERROR", AIMRT_RPC_STATUS_SVR_BACKEND_INTERNAL_ERROR)
      .value("SVR_NOT_IMPLEMENTED", AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED)
      .value("SVR_NOT_FOUND", AIMRT_RPC_STATUS_SVR_NOT_FOUND)
      .value("SVR_INVALID_SERIALIZATION_TYPE", AIMRT_RPC_STATUS_SVR_INVALID_SERIALIZATION_TYPE)
      .value("SVR_SERIALIZATION_FAILED", AIMRT_RPC_STATUS_SVR_SERIALIZATION_FAILED)
      .value("SVR_INVALID_DESERIALIZATION_TYPE", AIMRT_RPC_STATUS_SVR_INVALID_DESERIALIZATION_TYPE)
      .value("SVR_DESERIALIZATION_FAILED", AIMRT_RPC_STATUS_SVR_DESERIALIZATION_FAILED)
      .value("SVR_HANDLE_FAILED", AIMRT_RPC_STATUS_SVR_HANDLE_FAILED)
      // cli side
      .value("CLI_UNKNOWN", AIMRT_RPC_STATUS_CLI_UNKNOWN)
      .value("CLI_BACKEND_INTERNAL_ERROR", AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR)
      .value("CLI_INVALID_CONTEXT", AIMRT_RPC_STATUS_CLI_INVALID_CONTEXT)
      .value("CLI_INVALID_ADDR", AIMRT_RPC_STATUS_CLI_INVALID_ADDR)
      .value("CLI_INVALID_SERIALIZATION_TYPE", AIMRT_RPC_STATUS_CLI_INVALID_SERIALIZATION_TYPE)
      .value("CLI_SERIALIZATION_FAILED", AIMRT_RPC_STATUS_CLI_SERIALIZATION_FAILED)
      .value("CLI_INVALID_DESERIALIZATION_TYPE", AIMRT_RPC_STATUS_CLI_INVALID_DESERIALIZATION_TYPE)
      .value("CLI_DESERIALIZATION_FAILED", AIMRT_RPC_STATUS_CLI_DESERIALIZATION_FAILED)
      .value("CLI_NO_BACKEND_TO_HANDLE", AIMRT_RPC_STATUS_CLI_NO_BACKEND_TO_HANDLE)
      .value("CLI_SEND_REQ_FAILED", AIMRT_RPC_STATUS_CLI_SEND_REQ_FAILED)
      .value("CLI_FUNC_NOT_REGISTERED", AIMRT_RPC_STATUS_CLI_FUNC_NOT_REGISTERED);

  pybind11::class_<Status>(m, "RpcStatus")
      .def(pybind11::init<>())
      .def(pybind11::init<aimrt_rpc_status_code_t>())
      .def(pybind11::init<uint32_t>())
      .def("OK", &Status::OK)
      .def("__bool__", &Status::operator bool)
      .def("Code", &Status::Code)
      .def("ToString", &Status::ToString);
}

inline void ExportRpcContext(const pybind11::object& m) {
  using aimrt::rpc::Context;
  using aimrt::rpc::ContextRef;

  pybind11::enum_<aimrt_rpc_context_type_t>(m, "RpcContextType")
      .value("AIMRT_RPC_CLIENT_CONTEXT", AIMRT_RPC_CLIENT_CONTEXT)
      .value("AIMRT_RPC_SERVER_CONTEXT", AIMRT_RPC_SERVER_CONTEXT);

  pybind11::class_<Context, std::shared_ptr<Context>>(m, "RpcContext")
      .def(pybind11::init<>())
      .def("CheckUsed", &Context::CheckUsed)
      .def("SetUsed", &Context::SetUsed)
      .def("Reset", &Context::Reset)
      .def("GetType", &Context::GetType)
      .def("Timeout", &Context::Timeout)
      .def("SetTimeout", &Context::SetTimeout)
      .def("GetMetaValue", &Context::GetMetaValue)
      .def("SetMetaValue", &Context::SetMetaValue)
      .def("GetMetaKeys", &Context::GetMetaKeys)
      .def("GetToAddr", &Context::GetToAddr)
      .def("SetToAddr", &Context::SetToAddr)
      .def("GetSerializationType", &Context::GetSerializationType)
      .def("SetSerializationType", &Context::SetSerializationType)
      .def("GetFunctionName", &Context::GetFunctionName)
      .def("SetFunctionName", &Context::SetFunctionName)
      .def("ToString", &Context::ToString);

  pybind11::class_<ContextRef>(m, "RpcContextRef")
      .def(pybind11::init<>())
      .def(pybind11::init<const Context&>(), pybind11::keep_alive<1, 2>())
      .def(pybind11::init<Context*>(), pybind11::keep_alive<1, 2>())
      .def(pybind11::init<const std::shared_ptr<Context>&>(), pybind11::keep_alive<1, 2>())
      .def("__bool__", &ContextRef::operator bool)
      .def("CheckUsed", &ContextRef::CheckUsed)
      .def("SetUsed", &ContextRef::SetUsed)
      .def("GetType", &ContextRef::GetType)
      .def("Timeout", &ContextRef::Timeout)
      .def("SetTimeout", &ContextRef::SetTimeout)
      .def("GetMetaValue", &ContextRef::GetMetaValue)
      .def("SetMetaValue", &ContextRef::SetMetaValue)
      .def("GetMetaKeys", &ContextRef::GetMetaKeys)
      .def("GetToAddr", &ContextRef::GetToAddr)
      .def("SetToAddr", &ContextRef::SetToAddr)
      .def("GetSerializationType", &ContextRef::GetSerializationType)
      .def("SetSerializationType", &ContextRef::SetSerializationType)
      .def("GetFunctionName", &ContextRef::GetFunctionName)
      .def("SetFunctionName", &ContextRef::SetFunctionName)
      .def("ToString", &ContextRef::ToString);
}

using ServiceFuncReturnType = std::tuple<aimrt::rpc::Status, std::string>;
using ServiceFuncType = std::function<ServiceFuncReturnType(aimrt::rpc::ContextRef, const pybind11::bytes&)>;

inline void PbRpcServiceBaseRegisterServiceFunc(
    aimrt::rpc::ServiceBase& service,
    std::string_view func_name,
    const std::shared_ptr<const PyPbTypeSupport>& req_type_support,
    const std::shared_ptr<const PyPbTypeSupport>& rsp_type_support,
    ServiceFuncType&& service_func) {
  static std::vector<std::shared_ptr<const PyPbTypeSupport>> py_ts_vec;
  py_ts_vec.emplace_back(req_type_support);
  py_ts_vec.emplace_back(rsp_type_support);

  aimrt::rpc::ServiceFunc aimrt_service_func(
      [service_func{std::move(service_func)}](
          const aimrt_rpc_context_base_t* ctx, const void* req_ptr, void* rsp_ptr, aimrt_function_base_t* callback) {
        aimrt::rpc::ServiceCallback callback_f(callback);

        aimrt::rpc::ContextRef ctx_ref(ctx);

        try {
          const std::string& req_buf = *static_cast<const std::string*>(req_ptr);
          std::string& rsp_buf = *static_cast<std::string*>(rsp_ptr);

          pybind11::gil_scoped_acquire acquire;

          auto req_buf_bytes = pybind11::bytes(req_buf);
          auto [status, rsp_buf_tmp] = service_func(ctx_ref, req_buf_bytes);
          req_buf_bytes.release();

          pybind11::gil_scoped_release release;

          rsp_buf = std::move(rsp_buf_tmp);
          callback_f(status.Code());
          return;
        } catch (const std::exception& e) {
          callback_f(AIMRT_RPC_STATUS_SVR_HANDLE_FAILED);
          return;
        }
      });

  service.RegisterServiceFunc(
      func_name,
      nullptr,
      req_type_support->NativeHandle(),
      rsp_type_support->NativeHandle(),
      std::move(aimrt_service_func));
}

inline bool PbRpcHandleRefRegisterClientFunc(
    aimrt::rpc::RpcHandleRef& rpc_handle_ref,
    std::string_view func_name,
    const std::shared_ptr<const PyPbTypeSupport>& req_type_support,
    const std::shared_ptr<const PyPbTypeSupport>& rsp_type_support) {
  static std::vector<std::shared_ptr<const PyPbTypeSupport>> py_ts_vec;
  py_ts_vec.emplace_back(req_type_support);
  py_ts_vec.emplace_back(rsp_type_support);

  return rpc_handle_ref.RegisterClientFunc(
      func_name,
      nullptr,
      req_type_support->NativeHandle(),
      rsp_type_support->NativeHandle());
}

inline std::tuple<aimrt::rpc::Status, pybind11::bytes> PbRpcHandleRefInvoke(
    aimrt::rpc::RpcHandleRef& rpc_handle_ref,
    std::string_view func_name,
    aimrt::rpc::ContextRef ctx_ref,
    const std::string& req_buf) {
  pybind11::gil_scoped_release release;

  std::string rsp_buf;
  std::promise<uint32_t> status_promise;

  aimrt::rpc::ClientCallback callback([&status_promise](uint32_t status) {
    status_promise.set_value(status);
  });

  rpc_handle_ref.Invoke(
      func_name,
      ctx_ref,
      static_cast<const void*>(&req_buf),
      static_cast<void*>(&rsp_buf),
      std::move(callback));

  auto fu = status_promise.get_future();
  fu.wait();

  pybind11::gil_scoped_acquire acquire;

  return {aimrt::rpc::Status(fu.get()), pybind11::bytes(rsp_buf)};
}

#ifdef AIMRT_BUILD_WITH_ROS2

using Ros2ServiceFuncReturnType = std::tuple<aimrt::rpc::Status, pybind11::object>;
using Ros2ServiceFuncType = std::function<Ros2ServiceFuncReturnType(aimrt::rpc::ContextRef, pybind11::object)>;

inline void Ros2RpcServiceBaseRegisterServiceFunc(
    aimrt::rpc::ServiceBase& service,
    std::string_view func_name,
    pybind11::object srv_pyclass,
    const std::shared_ptr<const PyRos2TypeSupport>& req_type_support,
    pybind11::object req_pyclass,
    const std::shared_ptr<const PyRos2TypeSupport>& rsp_type_support,
    pybind11::object rsp_pyclass,
    Ros2ServiceFuncType&& service_func) {
  static std::vector<std::shared_ptr<const PyRos2TypeSupport>> py_ts_vec;
  py_ts_vec.emplace_back(req_type_support);
  py_ts_vec.emplace_back(rsp_type_support);

  pybind11::gil_scoped_acquire acquire;
  auto req_convert_to_py = get_convert_to_py_function(req_pyclass);
  auto rsp_convert_from_py = get_convert_from_py_function(rsp_pyclass);
  void* srv_type_support = common_get_type_support(srv_pyclass);
  pybind11::gil_scoped_release release;

  aimrt::rpc::ServiceFunc aimrt_service_func(
      [service_func{std::move(service_func)}, req_convert_to_py, rsp_convert_from_py](
          const aimrt_rpc_context_base_t* ctx, const void* req_ptr, void* rsp_ptr, aimrt_function_base_t* callback) {
        aimrt::rpc::ServiceCallback callback_f(callback);

        aimrt::rpc::ContextRef ctx_ref(ctx);

        try {
          pybind11::gil_scoped_acquire acquire;

          auto req_obj = pybind11::reinterpret_steal<pybind11::object>(req_convert_to_py(const_cast<void*>(req_ptr)));
          if (!req_obj) {
            throw py::error_already_set();
          }

          auto [status, rsp_obj] = service_func(ctx_ref, req_obj);

          if (!rsp_convert_from_py(rsp_obj.ptr(), rsp_ptr)) {
            throw py::error_already_set();
          }

          pybind11::gil_scoped_release release;

          callback_f(status.Code());
          return;
        } catch (const std::exception& e) {
          callback_f(AIMRT_RPC_STATUS_SVR_HANDLE_FAILED);
          return;
        }
      });

  service.RegisterServiceFunc(
      func_name,
      srv_type_support,
      req_type_support->NativeHandle(),
      rsp_type_support->NativeHandle(),
      std::move(aimrt_service_func));
}

inline bool PyRos2RpcHandleRefRegisterClientFunc(
    aimrt::rpc::RpcHandleRef& rpc_handle_ref,
    std::string_view func_name,
    pybind11::object srv_pyclass,
    const std::shared_ptr<const PyRos2TypeSupport>& req_type_support,
    const std::shared_ptr<const PyRos2TypeSupport>& rsp_type_support) {
  static std::vector<std::shared_ptr<const PyRos2TypeSupport>> py_ts_vec;
  py_ts_vec.emplace_back(req_type_support);
  py_ts_vec.emplace_back(rsp_type_support);

  pybind11::gil_scoped_acquire acquire;
  void* srv_type_support = common_get_type_support(srv_pyclass);
  pybind11::gil_scoped_release release;

  return rpc_handle_ref.RegisterClientFunc(
      func_name,
      srv_type_support,
      req_type_support->NativeHandle(),
      rsp_type_support->NativeHandle());
}

inline std::tuple<aimrt::rpc::Status, pybind11::object> Ros2RpcHandleRefInvoke(
    aimrt::rpc::RpcHandleRef& rpc_handle_ref,
    std::string_view func_name,
    aimrt::rpc::ContextRef ctx_ref,
    pybind11::object req,
    pybind11::object rsp_type) {
  auto req_ptr = convert_from_py(req);
  if (!req_ptr) {
    throw py::error_already_set();
  }

  static auto rsp_create = get_create_ros_message_function(rsp_type);
  auto* rsp_ptr = rsp_create();

  pybind11::gil_scoped_release release;

  std::promise<uint32_t> status_promise;
  aimrt::rpc::ClientCallback callback([&status_promise](uint32_t status) {
    status_promise.set_value(status);
  });

  rpc_handle_ref.Invoke(
      func_name,
      ctx_ref,
      static_cast<const void*>(req_ptr.get()),
      rsp_ptr,
      std::move(callback));

  auto fu = status_promise.get_future();
  fu.wait();

  pybind11::gil_scoped_acquire acquire;

  static auto rsp_convert_to_py = get_convert_to_py_function(rsp_type);

  auto rsp_obj = pybind11::reinterpret_steal<pybind11::object>(rsp_convert_to_py(rsp_ptr));
  if (!rsp_obj) {
    throw py::error_already_set();
  }

  return {aimrt::rpc::Status(fu.get()), rsp_obj};
}

#endif

inline void ExportRpcServiceBase(pybind11::object m) {
  using aimrt::rpc::ServiceBase;

  pybind11::class_<ServiceBase>(std::move(m), "ServiceBase")
      .def(pybind11::init<std::string_view, std::string_view>())
      .def("RpcType", &ServiceBase::RpcType)
      .def("ServiceName", &ServiceBase::ServiceName)
      .def("SetServiceName", &ServiceBase::SetServiceName)
      .def("PbRegisterServiceFunc", &PbRpcServiceBaseRegisterServiceFunc)
#ifdef AIMRT_BUILD_WITH_ROS2
      .def("Ros2RegisterServiceFunc", &Ros2RpcServiceBaseRegisterServiceFunc)
#endif
      ;
}

inline void ExportRpcHandleRef(pybind11::object m) {
  using aimrt::rpc::RpcHandleRef;

  pybind11::class_<RpcHandleRef>(std::move(m), "RpcHandleRef")
      .def(pybind11::init<>())
      .def("__bool__", &RpcHandleRef::operator bool)
      .def("RegisterService",
           pybind11::overload_cast<std::string_view, aimrt::rpc::ServiceBase*>(&RpcHandleRef::RegisterService))
      .def("RegisterService",
           pybind11::overload_cast<aimrt::rpc::ServiceBase*>(&RpcHandleRef::RegisterService))
      .def("PbRegisterClientFunc", &PbRpcHandleRefRegisterClientFunc)
      .def("PbInvoke", &PbRpcHandleRefInvoke)
#ifdef AIMRT_BUILD_WITH_ROS2
      .def("Ros2RegisterClientFunc", &PyRos2RpcHandleRefRegisterClientFunc)
      .def("Ros2Invoke", &Ros2RpcHandleRefInvoke)
#endif
      ;
}

inline void ExportRpcProxyBase(pybind11::object m) {
  using aimrt::rpc::ContextRef;
  using aimrt::rpc::ProxyBase;
  using aimrt::rpc::RpcHandleRef;

  pybind11::class_<ProxyBase>(std::move(m), "ProxyBase")
      .def(pybind11::init<RpcHandleRef, std::string_view, std::string_view>())
      .def("RpcType", &ProxyBase::RpcType)
      .def("ServiceName", &ProxyBase::ServiceName)
      .def("SetServiceName", &ProxyBase::SetServiceName)
      .def("NewContextSharedPtr", &ProxyBase::NewContextSharedPtr, pybind11::arg("ctx_ref") = ContextRef())
      .def("GetDefaultContextSharedPtr", &ProxyBase::GetDefaultContextSharedPtr)
      .def("SetDefaultContextSharedPtr", &ProxyBase::SetDefaultContextSharedPtr);
}

}  // namespace aimrt::runtime::python_runtime
