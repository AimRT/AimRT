// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "core/logger/logger_manager.h"

#include "log_control.aimrt_rpc.pb.h"

namespace aimrt::plugins::log_control_plugin {

class LogControlServiceImpl : public aimrt::protocols::log_control_plugin::LogControlServiceCoService {
 public:
  LogControlServiceImpl() = default;
  ~LogControlServiceImpl() override = default;

  void SetLoggerManager(aimrt::runtime::core::logger::LoggerManager* ptr) {
    logger_manager_ptr_ = ptr;
  }

  aimrt::co::Task<aimrt::rpc::Status> GetModuleLogLevel(
      aimrt::rpc::ContextRef ctx_ref,
      const ::aimrt::protocols::log_control_plugin::GetModuleLogLevelReq& req,
      ::aimrt::protocols::log_control_plugin::GetModuleLogLevelRsp& rsp) override;

  aimrt::co::Task<aimrt::rpc::Status> SetModuleLogLevel(
      aimrt::rpc::ContextRef ctx_ref,
      const ::aimrt::protocols::log_control_plugin::SetModuleLogLevelReq& req,
      ::aimrt::protocols::log_control_plugin::SetModuleLogLevelRsp& rsp) override;

 private:
  enum class ErrorCode : uint32_t {
    kSuc = 0,
    kInvalidModuleName = 1,
  };

  static constexpr std::string_view kErrorInfoArray[] = {
      "",
      "INVALID_MODULE_NAME"};

  template <typename T>
  void SetErrorCode(ErrorCode code, T& rsp) {
    rsp.set_code(static_cast<uint32_t>(code));
    rsp.set_msg(std::string(kErrorInfoArray[static_cast<uint32_t>(code)]));
  }

  aimrt::runtime::core::logger::LoggerManager* logger_manager_ptr_ = nullptr;
};

}  // namespace aimrt::plugins::log_control_plugin
