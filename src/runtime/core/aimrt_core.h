// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <future>
#include <string>
#include <vector>

#include "core/allocator/allocator_manager.h"
#include "core/channel/channel_manager.h"
#include "core/configurator/configurator_manager.h"
#include "core/executor/executor_manager.h"
#include "core/executor/guard_thread_executor.h"
#include "core/executor/main_thread_executor.h"
#include "core/logger/logger_manager.h"
#include "core/module/module_manager.h"
#include "core/parameter/parameter_manager.h"
#include "core/plugin/plugin_manager.h"
#include "core/rpc/rpc_manager.h"
#include "util/log_util.h"

namespace aimrt::runtime::core {

class AimRTCore {
 public:
  struct Options {
    std::string cfg_file_path;
  };

  enum class State : uint32_t {
    kPreInit,

    kPreInitConfigurator,
    kPostInitConfigurator,

    kPreInitPlugin,
    kPostInitPlugin,

    kPreInitMainThread,
    kPostInitMainThread,

    kPreInitGuardThread,
    kPostInitGuardThread,

    kPreInitExecutor,
    kPostInitExecutor,

    kPreInitLog,
    kPostInitLog,

    kPreInitAllocator,
    kPostInitAllocator,

    kPreInitRpc,
    kPostInitRpc,

    kPreInitChannel,
    kPostInitChannel,

    kPreInitParameter,
    kPostInitParameter,

    kPreInitModules,
    kPostInitModules,

    kPostInit,

    kPreStart,

    kPreStartConfigurator,
    kPostStartConfigurator,

    kPreStartPlugin,
    kPostStartPlugin,

    kPreStartMainThread,
    kPostStartMainThread,

    kPreStartGuardThread,
    kPostStartGuardThread,

    kPreStartExecutor,
    kPostStartExecutor,

    kPreStartLog,
    kPostStartLog,

    kPreStartAllocator,
    kPostStartAllocator,

    kPreStartRpc,
    kPostStartRpc,

    kPreStartChannel,
    kPostStartChannel,

    kPreStartParameter,
    kPostStartParameter,

    kPreStartModules,
    kPostStartModules,

    kPostStart,

    kPreShutdown,

    kPreShutdownModules,
    kPostShutdownModules,

    kPreShutdownParameter,
    kPostShutdownParameter,

    kPreShutdownChannel,
    kPostShutdownChannel,

    kPreShutdownRpc,
    kPostShutdownRpc,

    kPreShutdownAllocator,
    kPostShutdownAllocator,

    kPreShutdownLog,
    kPostShutdownLog,

    kPreShutdownExecutor,
    kPostShutdownExecutor,

    kPreShutdownGuardThread,
    kPostShutdownGuardThread,

    kPreShutdownMainThread,
    kPostShutdownMainThread,

    kPreShutdownPlugin,
    kPostShutdownPlugin,

    kPreShutdownConfigurator,
    kPostShutdownConfigurator,

    kPostShutdown,

    kMaxStateNum,
  };

  using HookTask = std::function<void()>;

 public:
  AimRTCore();
  ~AimRTCore();

  AimRTCore(const AimRTCore&) = delete;
  AimRTCore& operator=(const AimRTCore&) = delete;

  void Initialize(const Options& options);

  void Start();
  std::future<void> AsyncStart();

  void Shutdown();

  State GetState() const { return state_; }

  template <typename... Args>
    requires std::constructible_from<HookTask, Args...>
  void RegisterHookFunc(State state, Args&&... args) {
    hook_task_vec_array_[static_cast<uint32_t>(state)]
        .emplace_back(std::forward<Args>(args)...);
  }

  auto& GetConfiguratorManager() { return configurator_manager_; }
  auto& GetPluginManager() { return plugin_manager_; }
  auto& GetMainThreadExecutor() { return main_thread_executor_; }
  auto& GetGuardThreadExecutor() { return guard_thread_executor_; }
  auto& GetExecutorManager() { return executor_manager_; }
  auto& GetLoggerManager() { return logger_manager_; }
  auto& GetAllocatorManager() { return allocator_manager_; }
  auto& GetRpcManager() { return rpc_manager_; }
  auto& GetChannelManager() { return channel_manager_; }
  auto& GetParameterManager() { return parameter_manager_; }
  auto& GetModuleManager() { return module_manager_; }

  const auto& GetLogger() const { return *logger_ptr_; }

  std::string GenInitializationReport() const;

 private:
  void EnterState(State state);
  void SetCoreLogger();
  void ResetCoreLogger();
  aimrt::executor::ExecutorRef GetExecutor(std::string_view executor_name);
  void InitCoreProxy(const util::ModuleDetailInfo& info, module::CoreProxy& proxy);
  void CheckCfgFile() const;
  void StartImpl();
  void ShutdownImpl();
  void PrintStartupInfo() const;

 private:
  Options options_;
  State state_ = State::kPreInit;
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  std::atomic_bool shutdown_flag_ = false;
  std::promise<void> shutdown_promise_;
  std::atomic_bool shutdown_impl_flag_ = false;

  std::vector<std::vector<HookTask>> hook_task_vec_array_;

  configurator::ConfiguratorManager configurator_manager_;
  plugin::PluginManager plugin_manager_;
  executor::MainThreadExecutor main_thread_executor_;
  executor::GuardThreadExecutor guard_thread_executor_;
  executor::ExecutorManager executor_manager_;
  logger::LoggerManager logger_manager_;
  allocator::AllocatorManager allocator_manager_;
  rpc::RpcManager rpc_manager_;
  channel::ChannelManager channel_manager_;
  parameter::ParameterManager parameter_manager_;
  module::ModuleManager module_manager_;
};

}  // namespace aimrt::runtime::core
