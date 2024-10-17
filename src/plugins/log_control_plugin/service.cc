// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "log_control_plugin/service.h"
#include "core/logger/log_level_tool.h"
#include "log_control_plugin/global.h"

namespace aimrt::plugins::log_control_plugin {

aimrt::co::Task<aimrt::rpc::Status> LogControlServiceImpl::GetModuleLogLevel(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::log_control_plugin::GetModuleLogLevelReq& req,
    ::aimrt::protocols::log_control_plugin::GetModuleLogLevelRsp& rsp) {
  const auto& log_lvl_map = logger_manager_ptr_->GetAllLoggerLevels();

  // if empty, then get all module
  if (req.module_names().empty()) {
    for (const auto& itr : log_lvl_map) {
      rsp.mutable_module_log_level_map()->emplace(
          itr.first,
          std::string(aimrt::runtime::core::logger::LogLevelTool::GetLogLevelName(itr.second)));
    }

    co_return aimrt::rpc::Status();
  }

  for (const auto& module_name : req.module_names()) {
    auto finditr = log_lvl_map.find(module_name);
    if (finditr == log_lvl_map.end()) {
      SetErrorCode(ErrorCode::kInvalidModuleName, rsp);
      co_return aimrt::rpc::Status();
    }

    rsp.mutable_module_log_level_map()->emplace(
        finditr->first,
        std::string(aimrt::runtime::core::logger::LogLevelTool::GetLogLevelName(finditr->second)));
  }

  co_return aimrt::rpc::Status();
}

aimrt::co::Task<aimrt::rpc::Status> LogControlServiceImpl::SetModuleLogLevel(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::log_control_plugin::SetModuleLogLevelReq& req,
    ::aimrt::protocols::log_control_plugin::SetModuleLogLevelRsp& rsp) {
  std::unordered_map<std::string, aimrt_log_level_t> log_lvl_settings;

  for (const auto& itr : req.module_log_level_map()) {
    log_lvl_settings.emplace(
        itr.first,
        aimrt::runtime::core::logger::LogLevelTool::GetLogLevelFromName(itr.second));
  }

  logger_manager_ptr_->SetLoggerLevels(log_lvl_settings);

  co_return aimrt::rpc::Status();
}

}  // namespace aimrt::plugins::log_control_plugin
