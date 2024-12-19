// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <string>

#include "aimrt_module_c_interface/rpc/rpc_handle_base.h"
#include "aimrt_module_cpp_interface/rpc/rpc_co_filter.h"
#include "aimrt_module_cpp_interface/util/function.h"
#include "aimrt_module_cpp_interface/util/string.h"

namespace aimrt::rpc {

using ServiceCallback = aimrt::util::Function<aimrt_function_service_callback_ops_t>;
using ClientCallback = aimrt::util::Function<aimrt_function_client_callback_ops_t>;
using ServiceFunc = aimrt::util::Function<aimrt_function_service_func_ops_t>;

inline std::string GetFullFuncName(
    std::string_view rpc_type, std::string_view service_name, std::string_view func_name) {
  std::string full_name;

  full_name.reserve(rpc_type.size() + service_name.size() + func_name.size() + 3);
  full_name.append(rpc_type).append(":/").append(service_name).append("/").append(func_name);

  return full_name;
}

class ServiceBase {
  friend class RpcHandleRef;

 public:
  ServiceBase(std::string_view rpc_type, std::string_view service_name)
      : rpc_type_(rpc_type), service_name_(service_name) {}
  virtual ~ServiceBase() = default;

  ServiceBase(const ServiceBase&) = delete;
  ServiceBase& operator=(const ServiceBase&) = delete;

  void SetServiceName(std::string_view service_name) {
    service_name_ = service_name;
  }

  std::string_view RpcType() const {
    return rpc_type_;
  }

  std::string_view ServiceName() const {
    return service_name_;
  }

  void RegisterServiceFunc(
      std::string_view func_name,
      const void* custom_type_support_ptr,
      const aimrt_type_support_base_t* req_type_support,
      const aimrt_type_support_base_t* rsp_type_support,
      aimrt::rpc::ServiceFunc&& service_func) {
    service_func_wrapper_vec_.emplace_back(
        ServiceFuncWrapper{
            .func_name = std::string(func_name),
            .custom_type_support_ptr = custom_type_support_ptr,
            .req_type_support = req_type_support,
            .rsp_type_support = rsp_type_support,
            .service_func = std::move(service_func)});
  }

 protected:
  std::string rpc_type_;
  std::string service_name_;

  struct ServiceFuncWrapper {
    std::string func_name;
    const void* custom_type_support_ptr;
    const aimrt_type_support_base_t* req_type_support;
    const aimrt_type_support_base_t* rsp_type_support;
    aimrt::rpc::ServiceFunc service_func;
  };

  std::vector<ServiceFuncWrapper> service_func_wrapper_vec_;
};

class CoServiceBase : public ServiceBase {
 public:
  CoServiceBase(std::string_view rpc_type, std::string_view service_name)
      : ServiceBase(rpc_type, service_name) {}
  virtual ~CoServiceBase() = default;

  template <typename T>
    requires std::constructible_from<CoRpcFilter, T>
  void RegisterFilter(T&& filter) {
    filter_mgr_.RegisterFilter((T &&) filter);
  }

  auto& GetFilterManager() { return filter_mgr_; }

 protected:
  CoFilterManager filter_mgr_;
};

class RpcHandleRef {
 public:
  RpcHandleRef() = default;
  explicit RpcHandleRef(const aimrt_rpc_handle_base_t* base_ptr)
      : base_ptr_(base_ptr) {}
  ~RpcHandleRef() = default;

  explicit operator bool() const { return (base_ptr_ != nullptr); }

  const aimrt_rpc_handle_base_t* NativeHandle() const { return base_ptr_; }

  /**
   * @brief Register service
   *
   * @param service_ptr
   * @return Register result
   */
  bool RegisterService(ServiceBase* service_ptr) {
    AIMRT_ASSERT(base_ptr_, "Reference is null.");
    AIMRT_ASSERT(service_ptr, "Service pointer is null.");

    bool ret = true;

    for (auto& item : service_ptr->service_func_wrapper_vec_) {
      std::string full_func_name = GetFullFuncName(
          service_ptr->rpc_type_, service_ptr->service_name_, item.func_name);

      ret &= base_ptr_->register_service_func(
          base_ptr_->impl,
          aimrt::util::ToAimRTStringView(full_func_name),
          item.custom_type_support_ptr,
          item.req_type_support,
          item.rsp_type_support,
          item.service_func.NativeHandle());
    }

    return ret;
  }

  /**
   * @brief Register service with specific name
   *
   * @param service_name
   * @param service_ptr
   * @return Register result
   */
  bool RegisterService(std::string_view service_name, ServiceBase* service_ptr) {
    AIMRT_ASSERT(service_ptr, "Service pointer is null.");

    service_ptr->SetServiceName(service_name);

    return RegisterService(service_ptr);
  }

  /**
   * @brief Register client func
   *
   * @param full_func_name
   * @param custom_type_support_ptr
   * @param req_type_support
   * @param rsp_type_support
   * @return Register result
   */
  bool RegisterClientFunc(
      std::string_view full_func_name,
      const void* custom_type_support_ptr,
      const aimrt_type_support_base_t* req_type_support,
      const aimrt_type_support_base_t* rsp_type_support) {
    AIMRT_ASSERT(base_ptr_, "Reference is null.");
    return base_ptr_->register_client_func(
        base_ptr_->impl,
        aimrt::util::ToAimRTStringView(full_func_name),
        custom_type_support_ptr,
        req_type_support,
        rsp_type_support);
  }

  /**
   * @brief Register client func
   *
   * @param rpc_type
   * @param service_name
   * @param func_name
   * @param custom_type_support_ptr
   * @param req_type_support
   * @param rsp_type_support
   * @return Register result
   */
  bool RegisterClientFunc(
      std::string_view rpc_type,
      std::string_view service_name,
      std::string_view func_name,
      const void* custom_type_support_ptr,
      const aimrt_type_support_base_t* req_type_support,
      const aimrt_type_support_base_t* rsp_type_support) {
    return RegisterClientFunc(
        GetFullFuncName(rpc_type, service_name, func_name),
        custom_type_support_ptr, req_type_support, rsp_type_support);
  }

  /**
   * @brief Invoke rpc
   *
   * @param full_func_name
   * @param ctx_ref
   * @param req_ptr
   * @param rsp_ptr
   * @param callback
   */
  void Invoke(
      std::string_view full_func_name,
      ContextRef ctx_ref,
      const void* req_ptr,
      void* rsp_ptr,
      aimrt::rpc::ClientCallback&& callback) {
    AIMRT_ASSERT(base_ptr_, "Reference is null.");
    base_ptr_->invoke(
        base_ptr_->impl,
        aimrt::util::ToAimRTStringView(full_func_name),
        ctx_ref.NativeHandle(),
        req_ptr,
        rsp_ptr,
        callback.NativeHandle());
  }

  /**
   * @brief Invoke rpc
   *
   * @param rpc_type
   * @param service_name
   * @param func_name
   * @param ctx_ref
   * @param req_ptr
   * @param rsp_ptr
   * @param callback
   */
  void Invoke(
      std::string_view rpc_type,
      std::string_view service_name,
      std::string_view func_name,
      ContextRef ctx_ref,
      const void* req_ptr,
      void* rsp_ptr,
      aimrt::rpc::ClientCallback&& callback) {
    Invoke(
        GetFullFuncName(rpc_type, service_name, func_name),
        ctx_ref, req_ptr, rsp_ptr, std::move(callback));
  }

  void MergeServerContextToClientContext(
      const ContextRef server_ctx_ref, ContextRef client_ctx_ref) const {
    AIMRT_ASSERT(base_ptr_, "Reference is null.");
    base_ptr_->merge_server_context_to_client_context(
        base_ptr_->impl,
        server_ctx_ref.NativeHandle(),
        client_ctx_ref.NativeHandle());
  }

 private:
  const aimrt_rpc_handle_base_t* base_ptr_ = nullptr;
};

class ProxyBase {
 public:
  ProxyBase(RpcHandleRef rpc_handle_ref, std::string_view rpc_type, std::string_view service_name)
      : rpc_handle_ref_(rpc_handle_ref), rpc_type_(rpc_type), service_name_(service_name) {}
  virtual ~ProxyBase() = default;

  ProxyBase(const ProxyBase&) = delete;
  ProxyBase& operator=(const ProxyBase&) = delete;

  void SetServiceName(std::string_view service_name) {
    service_name_ = service_name;
  }

  std::string_view RpcType() const {
    return rpc_type_;
  }

  std::string_view ServiceName() const {
    return service_name_;
  }

  std::shared_ptr<Context> NewContextSharedPtr(ContextRef ctx_ref = ContextRef()) const {
    auto result_ctx = default_ctx_ptr_
                          ? std::make_shared<Context>(*default_ctx_ptr_)
                          : std::make_shared<Context>();
    if (ctx_ref) {
      rpc_handle_ref_.MergeServerContextToClientContext(ctx_ref, result_ctx);
    }

    return result_ctx;
  }

  void SetDefaultContextSharedPtr(const std::shared_ptr<Context>& ctx_ptr) {
    default_ctx_ptr_ = ctx_ptr;
  }

  std::shared_ptr<Context> GetDefaultContextSharedPtr() const {
    return default_ctx_ptr_;
  }

 protected:
  RpcHandleRef rpc_handle_ref_;
  std::string rpc_type_;
  std::string service_name_;
  std::shared_ptr<Context> default_ctx_ptr_;
};

class CoProxyBase : public ProxyBase {
 public:
  CoProxyBase(RpcHandleRef rpc_handle_ref, std::string_view rpc_type, std::string_view service_name)
      : ProxyBase(rpc_handle_ref, rpc_type, service_name) {}
  virtual ~CoProxyBase() = default;

  template <typename T>
    requires std::constructible_from<CoRpcFilter, T>
  void RegisterFilter(T&& filter) {
    filter_mgr_.RegisterFilter((T &&) filter);
  }

  auto& GetFilterManager() { return filter_mgr_; }

 protected:
  CoFilterManager filter_mgr_;
};

template <class ProxyType>
bool RegisterClientFunc(RpcHandleRef rpc_handle_ref) {
  return ProxyType::RegisterClientFunc(rpc_handle_ref);
}

template <class ProxyType>
bool RegisterClientFunc(RpcHandleRef rpc_handle_ref, std::string_view service_name) {
  return ProxyType::RegisterClientFunc(rpc_handle_ref, service_name);
}

}  // namespace aimrt::rpc
