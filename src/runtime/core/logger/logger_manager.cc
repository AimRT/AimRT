// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/logger/logger_manager.h"
#include "core/logger/console_logger_backend.h"
#include "core/logger/log_level_tool.h"
#include "core/logger/rotate_file_logger_backend.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::logger::LoggerManager::Options> {
  using Options = aimrt::runtime::core::logger::LoggerManager::Options;

  static Node encode(const Options& rhs) {
    Node node;
    node["core_lvl"] = aimrt::runtime::core::logger::LogLevelTool::GetLogLevelName(rhs.core_lvl);
    node["default_module_lvl"] = aimrt::runtime::core::logger::LogLevelTool::GetLogLevelName(rhs.default_module_lvl);

    node["backends"] = YAML::Node();
    for (const auto& backend_options : rhs.backends_options) {
      Node backend_options_node;
      backend_options_node["type"] = backend_options.type;
      backend_options_node["options"] = backend_options.options;
      node["backends"].push_back(backend_options_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["core_lvl"]) {
      rhs.core_lvl = aimrt::runtime::core::logger::LogLevelTool::GetLogLevelFromName(
          node["core_lvl"].as<std::string>());
    }

    if (node["default_module_lvl"]) {
      rhs.default_module_lvl = aimrt::runtime::core::logger::LogLevelTool::GetLogLevelFromName(
          node["default_module_lvl"].as<std::string>());
    }

    if (node["backends"] && node["backends"].IsSequence()) {
      for (const auto& backend_options_node : node["backends"]) {
        auto backend_options = Options::BackendOptions{
            .type = backend_options_node["type"].as<std::string>()};

        if (backend_options_node["options"])
          backend_options.options = backend_options_node["options"];

        rhs.backends_options.emplace_back(std::move(backend_options));
      }
    }
    return true;
  }
};
}  // namespace YAML

namespace aimrt::runtime::core::logger {

void LoggerManager::Initialize(YAML::Node options_node) {
  RegisterConsoleLoggerBackendGenFunc();
  RegisterRotateFileLoggerBackendGenFunc();

  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Logger manager can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  // 生成logger
  for (auto& backend_options : options_.backends_options) {
    auto finditr = logger_backend_gen_func_map_.find(backend_options.type);
    AIMRT_CHECK_ERROR_THROW(finditr != logger_backend_gen_func_map_.end(),
                            "Invalid logger backend type '{}'.",
                            backend_options.type);

    auto logger_backend_ptr = finditr->second();
    AIMRT_CHECK_ERROR_THROW(
        logger_backend_ptr,
        "Gen logger backend failed, logger backend type '{}'.",
        backend_options.type);

    if (!logger_backend_ptr->AllowDuplicates()) {
      AIMRT_CHECK_ERROR_THROW(
          std::find_if(logger_backend_vec_.begin(), logger_backend_vec_.end(),
                       [type = backend_options.type](const auto& backend_ptr) -> bool {
                         return backend_ptr->Type() == type;
                       }) == logger_backend_vec_.end(),
          "Logger backend type'{}' do not allow duplicate.", backend_options.type);
    }

    logger_backend_ptr->Initialize(backend_options.options);

    logger_backend_vec_.emplace_back(std::move(logger_backend_ptr));
  }

  AIMRT_CHECK_WARN(!logger_backend_vec_.empty(), "No log backend!");

  options_node = options_;

  AIMRT_INFO("Logger manager init complete");
}

void LoggerManager::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  for (auto& backend : logger_backend_vec_) {
    backend->Start();
  }

  AIMRT_INFO("Logger manager start completed.");
}

void LoggerManager::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  AIMRT_INFO("Logger manager shutdown.");

  // logger_proxy_map_不能清，有些插件还会打日志

  for (auto& backend : logger_backend_vec_) {
    backend->Shutdown();
  }

  // logger_backend不能清，可能会有未完成的日志任务

  logger_backend_gen_func_map_.clear();

  get_executor_func_ = std::function<executor::ExecutorRef(std::string_view)>();
}

void LoggerManager::RegisterGetExecutorFunc(
    const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  get_executor_func_ = get_executor_func;
}

void LoggerManager::RegisterLoggerBackendGenFunc(
    std::string_view type,
    LoggerBackendGenFunc&& logger_backend_gen_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  logger_backend_gen_func_map_.emplace(type, std::move(logger_backend_gen_func));
}

const LoggerProxy& LoggerManager::GetLoggerProxy(const util::ModuleDetailInfo& module_info) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit || state_.load() == State::kStart,
      "Method can only be called when state is 'Init' or 'Start'.");

  // module_name为空等效于aimrt节点
  const std::string& real_module_name =
      (module_info.name.empty()) ? "core" : module_info.name;

  auto itr = logger_proxy_map_.find(real_module_name);
  if (itr != logger_proxy_map_.end()) return *(itr->second);

  aimrt_log_level_t log_lvl =
      (real_module_name == "core")
          ? options_.core_lvl
          : (module_info.use_default_log_lvl ? options_.default_module_lvl
                                             : module_info.log_lvl);

  auto emplace_ret = logger_proxy_map_.emplace(
      real_module_name,
      std::make_unique<LoggerProxy>(real_module_name, log_lvl, logger_backend_vec_));

  return *(emplace_ret.first->second);
}

const LoggerProxy& LoggerManager::GetLoggerProxy(std::string_view logger_name) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit || state_.load() == State::kStart,
      "Method can only be called when state is 'Init' or 'Start'.");

  // logger_name为空等效于core节点
  const std::string& real_logger_name =
      (logger_name.empty()) ? "core" : std::string(logger_name);

  auto itr = logger_proxy_map_.find(real_logger_name);
  if (itr != logger_proxy_map_.end()) return *(itr->second);

  // 统一使用core_lvl
  auto emplace_ret = logger_proxy_map_.emplace(
      real_logger_name,
      std::make_unique<LoggerProxy>(real_logger_name, options_.core_lvl, logger_backend_vec_));

  return *(emplace_ret.first->second);
}

std::unordered_map<std::string, aimrt_log_level_t> LoggerManager::GetAllLoggerLevels() const {
  std::unordered_map<std::string, aimrt_log_level_t> result;
  for (const auto& itr : logger_proxy_map_) {
    result.emplace(itr.first, itr.second->LogLevel());
  }
  return result;
}

void LoggerManager::SetLoggerLevels(
    const std::unordered_map<std::string, aimrt_log_level_t>& logger_lvls) {
  for (const auto& itr : logger_lvls) {
    auto find_itr = logger_proxy_map_.find(itr.first);
    if (find_itr == logger_proxy_map_.end()) continue;

    find_itr->second->SetLogLevel(itr.second);
  }
}

void LoggerManager::RegisterConsoleLoggerBackendGenFunc() {
  RegisterLoggerBackendGenFunc("console", [this]() -> std::unique_ptr<LoggerBackendBase> {
    auto ptr = std::make_unique<ConsoleLoggerBackend>();
    ptr->RegisterGetExecutorFunc(get_executor_func_);
    return ptr;
  });
}

void LoggerManager::RegisterRotateFileLoggerBackendGenFunc() {
  RegisterLoggerBackendGenFunc("rotate_file", [this]() -> std::unique_ptr<LoggerBackendBase> {
    auto ptr = std::make_unique<RotateFileLoggerBackend>();
    ptr->RegisterGetExecutorFunc(get_executor_func_);
    return ptr;
  });
}

std::list<std::pair<std::string, std::string>> LoggerManager::GenInitializationReport() const {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  std::vector<std::string> logger_backend_type_vec;
  logger_backend_type_vec.reserve(logger_backend_vec_.size());
  for (const auto& logger_backend : logger_backend_vec_) {
    logger_backend_type_vec.emplace_back(logger_backend->Type());
  }

  std::string log_backend_name_list;
  if (logger_backend_type_vec.empty()) {
    log_backend_name_list = "<empty>";
  } else {
    log_backend_name_list = "[ " + aimrt::common::util::JoinVec(logger_backend_type_vec, " , ") + " ]";
  }

  std::list<std::pair<std::string, std::string>> report{
      {"Log Backend List", log_backend_name_list}};

  for (const auto& backend_ptr : logger_backend_vec_) {
    report.splice(report.end(), backend_ptr->GenInitializationReport());
  }

  return report;
}

const std::vector<std::unique_ptr<LoggerBackendBase>>& LoggerManager::GetUsedLoggerBackend() const {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");
  return logger_backend_vec_;
}
}  // namespace aimrt::runtime::core::logger
