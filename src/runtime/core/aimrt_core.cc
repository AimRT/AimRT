// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/aimrt_core.h"

#include <fstream>
#include <iostream>

#include "core/util/yaml_tools.h"

namespace aimrt::runtime::core {

AimRTCore::AimRTCore()
    : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {
  hook_task_vec_array_.resize(static_cast<uint32_t>(State::kMaxStateNum));
}

AimRTCore::~AimRTCore() {
  try {
    ShutdownImpl();
  } catch (const std::exception& e) {
    AIMRT_INFO("AimRTCore destruct get exception, {}", e.what());
  }

  hook_task_vec_array_.clear();
}

void AimRTCore::Initialize(const Options& options) {
  EnterState(State::kPreInit);

  options_ = options;

  // Init configurator
  EnterState(State::kPreInitConfigurator);
  configurator_manager_.SetLogger(logger_ptr_);
  configurator_manager_.Initialize(options_.cfg_file_path);
  EnterState(State::kPostInitConfigurator);

  // Init plugin
  EnterState(State::kPreInitPlugin);
  plugin_manager_.SetLogger(logger_ptr_);
  plugin_manager_.RegisterCorePtr(this);
  plugin_manager_.Initialize(configurator_manager_.GetAimRTOptionsNode("plugin"));
  EnterState(State::kPostInitPlugin);

  // Init main thread executor
  EnterState(State::kPreInitMainThread);
  main_thread_executor_.SetLogger(logger_ptr_);
  main_thread_executor_.Initialize(configurator_manager_.GetAimRTOptionsNode("main_thread"));
  EnterState(State::kPostInitMainThread);

  // Init guard thread executor
  EnterState(State::kPreInitGuardThread);
  guard_thread_executor_.SetLogger(logger_ptr_);
  guard_thread_executor_.Initialize(configurator_manager_.GetAimRTOptionsNode("guard_thread"));
  EnterState(State::kPostInitGuardThread);

  // Init executor
  EnterState(State::kPreInitExecutor);
  executor_manager_.SetLogger(logger_ptr_);
  executor_manager_.SetUsedExecutorName(main_thread_executor_.Name());
  executor_manager_.SetUsedExecutorName(guard_thread_executor_.Name());
  executor_manager_.Initialize(configurator_manager_.GetAimRTOptionsNode("executor"));
  EnterState(State::kPostInitExecutor);

  // Init log
  EnterState(State::kPreInitLog);
  logger_manager_.SetLogger(logger_ptr_);
  logger_manager_.RegisterGetExecutorFunc(
      std::bind(&AimRTCore::GetExecutor, this, std::placeholders::_1));
  logger_manager_.Initialize(configurator_manager_.GetAimRTOptionsNode("log"));
  SetCoreLogger();
  EnterState(State::kPostInitLog);

  // Init allocator
  EnterState(State::kPreInitAllocator);
  allocator_manager_.SetLogger(logger_ptr_);
  allocator_manager_.Initialize(configurator_manager_.GetAimRTOptionsNode("allocator"));
  EnterState(State::kPostInitAllocator);

  // Init rpc
  EnterState(State::kPreInitRpc);
  rpc_manager_.SetLogger(logger_ptr_);
  rpc_manager_.RegisterGetExecutorFunc(
      std::bind(&AimRTCore::GetExecutor, this, std::placeholders::_1));
  rpc_manager_.Initialize(configurator_manager_.GetAimRTOptionsNode("rpc"));
  EnterState(State::kPostInitRpc);

  // Init channel
  EnterState(State::kPreInitChannel);
  channel_manager_.SetLogger(logger_ptr_);
  channel_manager_.RegisterGetExecutorFunc(
      std::bind(&AimRTCore::GetExecutor, this, std::placeholders::_1));
  channel_manager_.Initialize(configurator_manager_.GetAimRTOptionsNode("channel"));
  EnterState(State::kPostInitChannel);

  // Init parameter
  EnterState(State::kPreInitParameter);
  parameter_manager_.SetLogger(logger_ptr_);
  parameter_manager_.Initialize(configurator_manager_.GetAimRTOptionsNode("parameter"));
  EnterState(State::kPostInitParameter);

  // Init modules
  EnterState(State::kPreInitModules);
  module_manager_.SetLogger(logger_ptr_);
  module_manager_.RegisterCoreProxyConfigurator(
      std::bind(&AimRTCore::InitCoreProxy, this, std::placeholders::_1, std::placeholders::_2));
  module_manager_.Initialize(configurator_manager_.GetAimRTOptionsNode("module"));
  EnterState(State::kPostInitModules);

  // Check cfg file
  CheckCfgFile();

  EnterState(State::kPostInit);
}

void AimRTCore::StartImpl() {
  AIMRT_INFO("Gen initialization report:\n{}", GenInitializationReport());

  EnterState(State::kPreStart);

  EnterState(State::kPreStartConfigurator);
  configurator_manager_.Start();
  EnterState(State::kPostStartConfigurator);

  EnterState(State::kPreStartPlugin);
  plugin_manager_.Start();
  EnterState(State::kPostStartPlugin);

  EnterState(State::kPreStartMainThread);
  main_thread_executor_.Start();
  EnterState(State::kPostStartMainThread);

  EnterState(State::kPreStartGuardThread);
  guard_thread_executor_.Start();
  EnterState(State::kPostStartGuardThread);

  EnterState(State::kPreStartExecutor);
  executor_manager_.Start();
  EnterState(State::kPostStartExecutor);

  EnterState(State::kPreStartLog);
  logger_manager_.Start();
  EnterState(State::kPostStartLog);

  EnterState(State::kPreStartAllocator);
  allocator_manager_.Start();
  EnterState(State::kPostStartAllocator);

  EnterState(State::kPreStartRpc);
  rpc_manager_.Start();
  EnterState(State::kPostStartRpc);

  EnterState(State::kPreStartChannel);
  channel_manager_.Start();
  EnterState(State::kPostStartChannel);

  EnterState(State::kPreStartParameter);
  parameter_manager_.Start();
  EnterState(State::kPostStartParameter);

  EnterState(State::kPreStartModules);
  module_manager_.Start();
  EnterState(State::kPostStartModules);

  EnterState(State::kPostStart);
}

void AimRTCore::ShutdownImpl() {
  if (std::atomic_exchange(&shutdown_impl_flag_, true)) return;

  EnterState(State::kPreShutdown);

  EnterState(State::kPreShutdownModules);
  module_manager_.Shutdown();
  EnterState(State::kPostShutdownModules);

  EnterState(State::kPreShutdownParameter);
  parameter_manager_.Shutdown();
  EnterState(State::kPostShutdownParameter);

  EnterState(State::kPreShutdownChannel);
  channel_manager_.Shutdown();
  EnterState(State::kPostShutdownChannel);

  EnterState(State::kPreShutdownRpc);
  rpc_manager_.Shutdown();
  EnterState(State::kPostShutdownRpc);

  EnterState(State::kPreShutdownAllocator);
  allocator_manager_.Shutdown();
  EnterState(State::kPostShutdownAllocator);

  EnterState(State::kPreShutdownLog);
  ResetCoreLogger();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  logger_manager_.Shutdown();
  EnterState(State::kPostShutdownLog);

  EnterState(State::kPreShutdownExecutor);
  executor_manager_.Shutdown();
  EnterState(State::kPostShutdownExecutor);

  EnterState(State::kPreShutdownGuardThread);
  guard_thread_executor_.Shutdown();
  EnterState(State::kPostShutdownGuardThread);

  EnterState(State::kPreShutdownMainThread);
  main_thread_executor_.Shutdown();
  EnterState(State::kPostShutdownMainThread);

  EnterState(State::kPreShutdownPlugin);
  plugin_manager_.Shutdown();
  EnterState(State::kPostShutdownPlugin);

  EnterState(State::kPreShutdownConfigurator);
  configurator_manager_.Shutdown();
  EnterState(State::kPostShutdownConfigurator);

  EnterState(State::kPostShutdown);
}

void AimRTCore::Start() {
  StartImpl();

  AIMRT_INFO("AimRT start completed, will waiting for shutdown.");

  shutdown_promise_.get_future().wait();
  ShutdownImpl();
}

std::future<void> AimRTCore::AsyncStart() {
  StartImpl();

  AIMRT_INFO("AimRT start completed, will waiting for async shutdown.");

  std::promise<void> end_running_promise;
  auto fu = end_running_promise.get_future();

  std::thread([this, end_running_promise{std::move(end_running_promise)}]() mutable {
    shutdown_promise_.get_future().wait();
    ShutdownImpl();
    end_running_promise.set_value();
  }).detach();

  return fu;
}

void AimRTCore::Shutdown() {
  if (std::atomic_exchange(&shutdown_flag_, true)) return;

  shutdown_promise_.set_value();
}

void AimRTCore::EnterState(State state) {
  state_ = state;
  for (const auto& func : hook_task_vec_array_[static_cast<uint32_t>(state)])
    func();
}

void AimRTCore::SetCoreLogger() {
  const auto* core_logger_ptr = logger_manager_.GetLoggerProxy("core").NativeHandle();

  logger_ptr_->get_log_level_func = [core_logger_ptr]() -> uint32_t {
    return core_logger_ptr->get_log_level(core_logger_ptr->impl);
  };

  logger_ptr_->log_func =
      [core_logger_ptr](
          uint32_t lvl,
          uint32_t line,
          uint32_t column,
          const char* file_name,
          const char* function_name,
          const char* log_data,
          size_t log_data_size) {
        core_logger_ptr->log(
            core_logger_ptr->impl,
            static_cast<aimrt_log_level_t>(lvl),
            line, column, file_name, function_name,
            log_data, log_data_size);  //
      };
}

void AimRTCore::ResetCoreLogger() {
  logger_ptr_->get_log_level_func = aimrt::common::util::SimpleLogger::GetLogLevel;
  logger_ptr_->log_func = aimrt::common::util::SimpleLogger::Log;
}

aimrt::executor::ExecutorRef AimRTCore::GetExecutor(std::string_view executor_name) {
  if (executor_name.empty() || executor_name == guard_thread_executor_.Name())
    return aimrt::executor::ExecutorRef(guard_thread_executor_.NativeHandle());
  return GetExecutorManager().GetExecutor(executor_name);
}

void AimRTCore::InitCoreProxy(const util::ModuleDetailInfo& info, module::CoreProxy& proxy) {
  proxy.SetConfigurator(configurator_manager_.GetConfiguratorProxy(info).NativeHandle());
  proxy.SetExecutorManager(executor_manager_.GetExecutorManagerProxy(info).NativeHandle());
  proxy.SetLogger(logger_manager_.GetLoggerProxy(info).NativeHandle());
  proxy.SetAllocator(allocator_manager_.GetAllocatorProxy(info).NativeHandle());
  proxy.SetRpcHandle(rpc_manager_.GetRpcHandleProxy(info).NativeHandle());
  proxy.SetChannelHandle(channel_manager_.GetChannelHandleProxy(info).NativeHandle());
  proxy.SetParameterHandle(parameter_manager_.GetParameterHandleProxy(info).NativeHandle());
}

void AimRTCore::CheckCfgFile() const {
  std::string check_msg = util::CheckYamlNodes(
      configurator_manager_.GetRootOptionsNode()["aimrt"],
      configurator_manager_.GetUserRootOptionsNode()["aimrt"],
      "aimrt");

  if (!check_msg.empty())
    AIMRT_WARN("Configuration Name Warning in \"{}\":\n{}",
               configurator_manager_.GetConfigureFilePath(), check_msg);
}

std::string AimRTCore::GenInitializationReport() const {
  AIMRT_CHECK_ERROR_THROW(state_ == State::kPostInit,
                          "Initialization report can only be generated after initialization is complete.");

  std::list<std::pair<std::string, std::string>> report;

  report.splice(report.end(), configurator_manager_.GenInitializationReport());
  report.splice(report.end(), plugin_manager_.GenInitializationReport());
  report.splice(report.end(), executor_manager_.GenInitializationReport());
  report.splice(report.end(), logger_manager_.GenInitializationReport());
  report.splice(report.end(), allocator_manager_.GenInitializationReport());
  report.splice(report.end(), rpc_manager_.GenInitializationReport());
  report.splice(report.end(), channel_manager_.GenInitializationReport());
  report.splice(report.end(), parameter_manager_.GenInitializationReport());
  report.splice(report.end(), module_manager_.GenInitializationReport());

  std::stringstream result;
  result << "\n----------------------- AimRT Initialization Report Begin ----------------------\n\n";

  size_t count = 0;
  for (auto& itr : report) {
    ++count;
    result << "[" << count << "]. " << itr.first << "\n"
           << itr.second << "\n\n";
  }

  result << "\n----------------------- AimRT Initialization Report End ------------------------\n\n";

  return result.str();
}

}  // namespace aimrt::runtime::core
