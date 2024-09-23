// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <unordered_map>

#include "time_manipulator_plugin/time_manipulator_executor.h"

#include "time_manipulator.aimrt_rpc.pb.h"

namespace aimrt::plugins::time_manipulator_plugin {

class TimeManipulatorServiceImpl : public aimrt::protocols::time_manipulator_plugin::TimeManipulatorServiceCoService {
 public:
  TimeManipulatorServiceImpl() = default;
  ~TimeManipulatorServiceImpl() override = default;

  void RegisterTimeManipulatorExecutor(TimeManipulatorExecutor* ptr) {
    executor_map_.emplace(ptr->Name(), ptr);
  }

  aimrt::co::Task<aimrt::rpc::Status> SetTimeRatio(
      aimrt::rpc::ContextRef ctx_ref,
      const ::aimrt::protocols::time_manipulator_plugin::SetTimeRatioReq& req,
      ::aimrt::protocols::time_manipulator_plugin::CommonRsp& rsp) override;

  aimrt::co::Task<aimrt::rpc::Status> Pause(
      aimrt::rpc::ContextRef ctx_ref,
      const ::aimrt::protocols::time_manipulator_plugin::PauseReq& req,
      ::aimrt::protocols::time_manipulator_plugin::CommonRsp& rsp) override;

  aimrt::co::Task<aimrt::rpc::Status> GetTimeRatio(
      aimrt::rpc::ContextRef ctx_ref,
      const ::aimrt::protocols::time_manipulator_plugin::GetTimeRatioReq& req,
      ::aimrt::protocols::time_manipulator_plugin::CommonRsp& rsp) override;

 private:
  enum class ErrorCode : uint32_t {
    kSuc = 0,
    kInvalidExecutorName = 1,
  };

  static constexpr std::string_view kErrorInfoArray[] = {
      "",
      "INVALID_EXECUTOR_NAME"};

  static void SetErrorCode(
      ErrorCode code,
      ::aimrt::protocols::time_manipulator_plugin::CommonRsp& rsp) {
    rsp.set_code(static_cast<uint32_t>(code));
    rsp.set_msg(std::string(kErrorInfoArray[static_cast<uint32_t>(code)]));
  }

  std::unordered_map<std::string_view, TimeManipulatorExecutor*> executor_map_;
};

}  // namespace aimrt::plugins::time_manipulator_plugin
