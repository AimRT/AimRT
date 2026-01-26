// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>

#include "aimrt_module_c_interface/util/function_base.h"
#include "core/rpc/rpc_backend_base.h"
#include "core/rpc/rpc_framework_async_filter.h"
#include "util/log_util.h"

namespace aimrt::runtime::core::rpc {

struct RegisterServiceFuncProxyInfoWrapper {
  std::string_view pkg_path;
  std::string_view module_name;

  aimrt_string_view_t func_name;
  const void* custom_type_support_ptr;
  const aimrt_type_support_base_t* req_type_support;
  const aimrt_type_support_base_t* rsp_type_support;
  aimrt_function_base_t* service_func;
};

struct RegisterClientFuncProxyInfoWrapper {
  std::string_view pkg_path;
  std::string_view module_name;

  aimrt_string_view_t func_name;
  const void* custom_type_support_ptr;
  const aimrt_type_support_base_t* req_type_support;
  const aimrt_type_support_base_t* rsp_type_support;
};

struct InvokeProxyInfoWrapper {
  std::string_view pkg_path;
  std::string_view module_name;

  aimrt_string_view_t func_name;
  const aimrt_rpc_context_base_t* ctx_ptr;
  const void* req_ptr;
  void* rsp_ptr;
  aimrt_function_base_t* callback;
};

class RpcBackendManager {
 public:
  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 public:
  RpcBackendManager()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~RpcBackendManager() = default;

  RpcBackendManager(const RpcBackendManager&) = delete;
  RpcBackendManager& operator=(const RpcBackendManager&) = delete;

  void Initialize();
  void Start();
  void Shutdown();

  State GetState() const { return state_.load(); }

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  void SetRpcRegistry(RpcRegistry* rpc_registry_ptr);

  void SetClientsFiltersRules(
      const std::vector<std::pair<std::string, std::vector<std::string>>>& rules);
  void SetServersFiltersRules(
      const std::vector<std::pair<std::string, std::vector<std::string>>>& rules);

  void SetClientFrameworkAsyncRpcFilterManager(FrameworkAsyncRpcFilterManager* ptr);
  void SetServerFrameworkAsyncRpcFilterManager(FrameworkAsyncRpcFilterManager* ptr);

  void SetClientsBackendsRules(
      const std::vector<std::pair<std::string, std::vector<std::string>>>& rules);
  void SetServersBackendsRules(
      const std::vector<std::pair<std::string, std::vector<std::string>>>& rules);

  void RegisterRpcBackend(RpcBackendBase* rpc_backend_ptr);

  bool RegisterServiceFunc(RegisterServiceFuncProxyInfoWrapper&& wrapper);
  bool RegisterClientFunc(RegisterClientFuncProxyInfoWrapper&& wrapper);
  void Invoke(InvokeProxyInfoWrapper&& wrapper);

  using FuncBackendInfoMap = std::unordered_map<std::string_view, std::vector<std::string_view>>;
  FuncBackendInfoMap GetClientsBackendInfo() const;
  FuncBackendInfoMap GetServersBackendInfo() const;

 private:
  std::vector<RpcBackendBase*> GetBackendsByRules(
      std::string_view func_name,
      const std::vector<std::pair<std::string, std::vector<std::string>>>& rules);

  std::vector<std::string> GetFilterRules(
      std::string_view func_name,
      const std::vector<std::pair<std::string, std::vector<std::string>>>& rules);

 private:
  std::atomic<State> state_ = State::kPreInit;
  uint64_t cli_index_ = 0;
  uint64_t srv_index_ = 0;

  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  RpcRegistry* rpc_registry_ptr_ = nullptr;

  // filter
  std::vector<std::pair<std::string, std::vector<std::string>>> clients_filters_rules_;
  std::vector<std::pair<std::string, std::vector<std::string>>> servers_filters_rules_;

  FrameworkAsyncRpcFilterManager* client_filter_manager_ptr_ = nullptr;
  FrameworkAsyncRpcFilterManager* server_filter_manager_ptr_ = nullptr;

  // backend
  std::vector<RpcBackendBase*> rpc_backend_index_vec_;
  std::unordered_map<std::string_view, RpcBackendBase*> rpc_backend_index_map_;

  std::vector<std::pair<std::string, std::vector<std::string>>> clients_backends_rules_;
  std::vector<std::pair<std::string, std::vector<std::string>>> servers_backends_rules_;

  std::unordered_map<
      std::string,
      std::vector<RpcBackendBase*>,
      aimrt::common::util::StringHash,
      std::equal_to<>>
      clients_backend_index_map_;

  std::unordered_map<
      std::string,
      std::vector<RpcBackendBase*>,
      aimrt::common::util::StringHash,
      std::equal_to<>>
      servers_backend_index_map_;

  std::unordered_map<
      std::string,
      std::atomic<uint32_t>,
      aimrt::common::util::StringHash,
      std::equal_to<>>
      request_id_map_;
};
}  // namespace aimrt::runtime::core::rpc
