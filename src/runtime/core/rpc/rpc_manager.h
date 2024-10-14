// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "aimrt_module_c_interface/rpc/rpc_handle_base.h"
#include "aimrt_module_cpp_interface/executor/executor.h"
#include "core/rpc/rpc_backend_manager.h"
#include "core/rpc/rpc_handle_proxy.h"
#include "core/util/module_detail_info.h"
#include "util/log_util.h"

namespace aimrt::runtime::core::rpc {

class RpcManager {
 public:
  struct Options {
    struct BackendOptions {
      std::string type;
      YAML::Node options;
    };
    std::vector<BackendOptions> backends_options;

    struct ClientOptions {
      std::string func_name;
      std::vector<std::string> enable_backends;
      std::vector<std::string> enable_filters;
    };
    std::vector<ClientOptions> clients_options;

    struct ServerOptions {
      std::string func_name;
      std::vector<std::string> enable_backends;
      std::vector<std::string> enable_filters;
    };
    std::vector<ServerOptions> servers_options;
  };

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 public:
  RpcManager()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~RpcManager() = default;

  RpcManager(const RpcManager&) = delete;
  RpcManager& operator=(const RpcManager&) = delete;

  void Initialize(YAML::Node options_node);
  void Start();
  void Shutdown();

  const Options& GetOptions() const { return options_; }
  State GetState() const { return state_.load(); }

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  std::list<std::pair<std::string, std::string>> GenInitializationReport() const;

  void RegisterRpcBackend(std::unique_ptr<RpcBackendBase>&& rpc_backend_ptr);

  void RegisterGetExecutorFunc(
      const std::function<aimrt::executor::ExecutorRef(std::string_view)>& get_executor_func);

  const RpcHandleProxy& GetRpcHandleProxy(const util::ModuleDetailInfo& module_info);
  const RpcHandleProxy& GetRpcHandleProxy(std::string_view module_name = "core") {
    return GetRpcHandleProxy(util::ModuleDetailInfo{.name = std::string(module_name), .pkg_path = "core"});
  }

  void RegisterClientFilter(std::string_view name, FrameworkAsyncRpcFilter&& filter);
  void RegisterServerFilter(std::string_view name, FrameworkAsyncRpcFilter&& filter);

  void AddPassedContextMetaKeys(const std::unordered_set<std::string>& keys);

  const RpcRegistry* GetRpcRegistry() const;
  const std::vector<RpcBackendBase*>& GetUsedRpcBackend() const;

 private:
  void RegisterLocalRpcBackend();
  void RegisterDebugLogFilter();

 private:
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  std::function<aimrt::executor::ExecutorRef(std::string_view)> get_executor_func_;

  std::unordered_set<std::string> passed_context_meta_keys_;

  FrameworkAsyncRpcFilterManager client_filter_manager_;
  FrameworkAsyncRpcFilterManager server_filter_manager_;

  std::unique_ptr<RpcRegistry> rpc_registry_ptr_;

  std::vector<std::unique_ptr<RpcBackendBase>> rpc_backend_vec_;
  std::vector<RpcBackendBase*> used_rpc_backend_vec_;

  RpcBackendManager rpc_backend_manager_;

  std::unordered_map<std::string, std::unique_ptr<RpcHandleProxy>> rpc_handle_proxy_map_;
};

}  // namespace aimrt::runtime::core::rpc
