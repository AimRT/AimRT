// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include <pybind11/pybind11.h>

#include <pybind11/functional.h>  // IWYU pragma: keep
#include <pybind11/stl.h>         // IWYU pragma: keep

#include "aimrt_module_c_interface/rpc/rpc_context_base.h"
#include "aimrt_module_cpp_interface/rpc/rpc_handle.h"

#include "python_runtime/export_ros2_type_support.h"

namespace aimrt::runtime::python_runtime {

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
        } catch (...) {
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

  auto rsp_create = get_create_ros_message_function(rsp_type);
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

  auto rsp_convert_to_py = get_convert_to_py_function(rsp_type);

  auto rsp_obj = pybind11::reinterpret_steal<pybind11::object>(rsp_convert_to_py(rsp_ptr));
  if (!rsp_obj) {
    throw py::error_already_set();
  }

  return {aimrt::rpc::Status(fu.get()), rsp_obj};
}

inline void ExportRos2RpcServiceFunc(pybind11::module& m) {
  m.def("Ros2RegisterServiceFunc", &Ros2RpcServiceBaseRegisterServiceFunc);
}

inline void ExportRos2ClientFunc(pybind11::module& m) {
  m.def("Ros2RegisterClientFunc", &PyRos2RpcHandleRefRegisterClientFunc);
  m.def("Ros2Invoke", &Ros2RpcHandleRefInvoke);
}

}  // namespace aimrt::runtime::python_runtime