// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <future>
#include <string>
#include <utility>

#include "aimrt_module_cpp_interface/rpc/rpc_handle.h"
#include "python_runtime/export_type_support.h"

#include "pybind11/pybind11.h"

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

  pybind11::class_<Context>(m, "RpcContext")
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
      .def(pybind11::init<const Context&>())
      .def(pybind11::init<Context*>())
      .def(pybind11::init<const std::shared_ptr<Context>&>())
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

inline void PyRpcServiceBaseRegisterServiceFunc(
    aimrt::rpc::ServiceBase& service,
    std::string_view func_name,
    const std::shared_ptr<const PyTypeSupport>& req_type_support,
    const std::shared_ptr<const PyTypeSupport>& rsp_type_support,
    std::function<std::tuple<aimrt::rpc::Status, std::string>(aimrt::rpc::ContextRef, const pybind11::bytes&)>&& service_func) {
  static std::vector<std::shared_ptr<const PyTypeSupport>> py_ts_vec;
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

inline void ExportRpcServiceBase(pybind11::object m) {
  using aimrt::rpc::ServiceBase;

  pybind11::class_<ServiceBase>(std::move(m), "ServiceBase")
      .def(pybind11::init<std::string_view, std::string_view>())
      .def("RpcType", &ServiceBase::RpcType)
      .def("ServiceName", &ServiceBase::ServiceName)
      .def("SetServiceName", &ServiceBase::SetServiceName)
      .def("RegisterServiceFunc", &PyRpcServiceBaseRegisterServiceFunc);
}

inline bool PyRpcHandleRefRegisterClientFunc(
    aimrt::rpc::RpcHandleRef& rpc_handle_ref,
    std::string_view func_name,
    const std::shared_ptr<const PyTypeSupport>& req_type_support,
    const std::shared_ptr<const PyTypeSupport>& rsp_type_support) {
  static std::vector<std::shared_ptr<const PyTypeSupport>> py_ts_vec;
  py_ts_vec.emplace_back(req_type_support);
  py_ts_vec.emplace_back(rsp_type_support);

  return rpc_handle_ref.RegisterClientFunc(
      func_name,
      nullptr,
      req_type_support->NativeHandle(),
      rsp_type_support->NativeHandle());
}

inline std::tuple<aimrt::rpc::Status, pybind11::bytes> PyRpcHandleRefInvoke(
    aimrt::rpc::RpcHandleRef& rpc_handle_ref,
    std::string_view func_name,
    aimrt::rpc::ContextRef ctx_ref,
    const std::string& req_buf) {
  pybind11::gil_scoped_release release;

  std::string rsp_buf;
  std::promise<uint32_t> status_promise;

  aimrt::rpc::ClientCallback callback(
      [&status_promise](uint32_t status) {
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

inline void ExportRpcHandleRef(pybind11::object m) {
  using aimrt::rpc::RpcHandleRef;

  pybind11::class_<RpcHandleRef>(std::move(m), "RpcHandleRef")
      .def(pybind11::init<>())
      .def("__bool__", &RpcHandleRef::operator bool)
      .def("RegisterService",
           pybind11::overload_cast<std::string_view, aimrt::rpc::ServiceBase*>(&RpcHandleRef::RegisterService))
      .def("RegisterService",
           pybind11::overload_cast<aimrt::rpc::ServiceBase*>(&RpcHandleRef::RegisterService))
      .def("RegisterClientFunc", &PyRpcHandleRefRegisterClientFunc)
      .def("Invoke", &PyRpcHandleRefInvoke);
}
}  // namespace aimrt::runtime::python_runtime
