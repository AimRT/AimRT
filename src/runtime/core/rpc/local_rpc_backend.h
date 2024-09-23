// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <unordered_set>

#include "core/rpc/rpc_backend_base.h"
#include "core/util/rpc_client_tool.h"
#include "util/log_util.h"

namespace aimrt::runtime::core::rpc {

class LocalRpcBackend : public RpcBackendBase {
 public:
  struct Options {
    std::string timeout_executor;
  };

  enum class State : uint32_t {
    kPreInit,
    kInit,
    kStart,
    kShutdown,
  };

 public:
  LocalRpcBackend()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~LocalRpcBackend() override = default;

  std::string_view Name() const noexcept override { return "local"; }

  void Initialize(YAML::Node options_node) override;
  void Start() override;
  void Shutdown() override;

  const Options& GetOptions() const { return options_; }
  State GetState() const { return state_.load(); }

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  void SetRpcRegistry(const RpcRegistry* rpc_registry_ptr) noexcept override {
    rpc_registry_ptr_ = rpc_registry_ptr;
  }

  bool RegisterServiceFunc(
      const ServiceFuncWrapper& service_func_wrapper) noexcept override;
  bool RegisterClientFunc(
      const ClientFuncWrapper& client_func_wrapper) noexcept override;
  void Invoke(
      const std::shared_ptr<InvokeWrapper>& client_invoke_wrapper_ptr) noexcept override;

  void RegisterGetExecutorFunc(
      const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func);

 private:
  Options options_;
  std::atomic<State> state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  const RpcRegistry* rpc_registry_ptr_ = nullptr;

  std::function<executor::ExecutorRef(std::string_view)> get_executor_func_;

  std::atomic_uint32_t req_id_ = 0;

  std::unique_ptr<util::RpcClientTool<std::shared_ptr<InvokeWrapper>>> client_tool_ptr_;

  using ServiceFuncIndexMap =
      std::unordered_map<
          std::string_view,  // func_name
          std::unordered_map<
              std::string_view,  // lib_path
              std::unordered_set<
                  std::string_view>>>;  // module_name
  ServiceFuncIndexMap service_func_register_index_;
};

}  // namespace aimrt::runtime::core::rpc
