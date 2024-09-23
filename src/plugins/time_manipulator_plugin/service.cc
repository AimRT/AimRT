// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "time_manipulator_plugin/service.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "time_manipulator_plugin/global.h"

namespace aimrt::plugins::time_manipulator_plugin {

aimrt::co::Task<aimrt::rpc::Status> TimeManipulatorServiceImpl::SetTimeRatio(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::time_manipulator_plugin::SetTimeRatioReq& req,
    ::aimrt::protocols::time_manipulator_plugin::CommonRsp& rsp) {
  auto itr = executor_map_.find(req.executor_name());
  if (itr == executor_map_.end()) [[unlikely]] {
    SetErrorCode(ErrorCode::kInvalidExecutorName, rsp);
    co_return aimrt::rpc::Status();
  }

  itr->second->SetTimeRatio(req.time_ratio());
  rsp.set_time_ratio(itr->second->GetTimeRatio());

  co_return aimrt::rpc::Status();
}

aimrt::co::Task<aimrt::rpc::Status> TimeManipulatorServiceImpl::Pause(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::time_manipulator_plugin::PauseReq& req,
    ::aimrt::protocols::time_manipulator_plugin::CommonRsp& rsp) {
  auto itr = executor_map_.find(req.executor_name());
  if (itr == executor_map_.end()) [[unlikely]] {
    SetErrorCode(ErrorCode::kInvalidExecutorName, rsp);
    co_return aimrt::rpc::Status();
  }

  itr->second->SetTimeRatio(0.0);
  rsp.set_time_ratio(itr->second->GetTimeRatio());

  co_return aimrt::rpc::Status();
}

aimrt::co::Task<aimrt::rpc::Status> TimeManipulatorServiceImpl::GetTimeRatio(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::time_manipulator_plugin::GetTimeRatioReq& req,
    ::aimrt::protocols::time_manipulator_plugin::CommonRsp& rsp) {
  auto itr = executor_map_.find(req.executor_name());
  if (itr == executor_map_.end()) [[unlikely]] {
    SetErrorCode(ErrorCode::kInvalidExecutorName, rsp);
    co_return aimrt::rpc::Status();
  }

  rsp.set_time_ratio(itr->second->GetTimeRatio());

  co_return aimrt::rpc::Status();
}

}  // namespace aimrt::plugins::time_manipulator_plugin
