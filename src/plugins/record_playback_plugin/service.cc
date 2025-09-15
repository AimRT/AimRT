// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "record_playback_plugin/service.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "record_playback_plugin/global.h"

namespace aimrt::plugins::record_playback_plugin {

aimrt::co::Task<aimrt::rpc::Status> RecordPlaybackServiceImpl::StartRecord(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::record_playback_plugin::StartRecordReq& req,
    ::aimrt::protocols::record_playback_plugin::StartRecordRsp& rsp) {
  auto finditr = record_action_map_ptr_->find(req.action_name());
  auto common_rsp = rsp.mutable_common_rsp();

  if (finditr == record_action_map_ptr_->end()) {
    SetErrorCode(ErrorCode::kInvalidActionName, *common_rsp);
    co_return aimrt::rpc::Status();
  }

  auto& action_wrapper = *(finditr->second);

  if (action_wrapper.GetOptions().mode != RecordAction::Options::Mode::kSignal) {
    SetErrorCode(ErrorCode::kInvalidActionMode, *common_rsp);
    co_return aimrt::rpc::Status();
  }

  uint64_t preparation_duration_s = req.preparation_duration_s();
  uint64_t record_duration_s = req.record_duration_s();

  std::string filefolder;
  bool ret = action_wrapper.StartSignalRecord(preparation_duration_s, record_duration_s, filefolder);
  if (!ret) {
    SetErrorCode(ErrorCode::kStartRecordFailed, *common_rsp);
    co_return aimrt::rpc::Status();
  }

  rsp.set_filefolder(filefolder);

  co_return aimrt::rpc::Status();
}

aimrt::co::Task<aimrt::rpc::Status> RecordPlaybackServiceImpl::StopRecord(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::record_playback_plugin::StopRecordReq& req,
    ::aimrt::protocols::record_playback_plugin::CommonRsp& rsp) {
  auto finditr = record_action_map_ptr_->find(req.action_name());
  if (finditr == record_action_map_ptr_->end()) {
    SetErrorCode(ErrorCode::kInvalidActionName, rsp);
    co_return aimrt::rpc::Status();
  }

  auto& action_wrapper = *(finditr->second);

  if (action_wrapper.GetOptions().mode != RecordAction::Options::Mode::kSignal) {
    SetErrorCode(ErrorCode::kInvalidActionMode, rsp);
    co_return aimrt::rpc::Status();
  }

  bool ret = action_wrapper.StopSignalRecord();
  if (!ret) {
    SetErrorCode(ErrorCode::kStopRecordFailed, rsp);
    co_return aimrt::rpc::Status();
  }

  co_return aimrt::rpc::Status();
}

aimrt::co::Task<aimrt::rpc::Status> RecordPlaybackServiceImpl::StartPlayback(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::record_playback_plugin::StartPlaybackReq& req,
    ::aimrt::protocols::record_playback_plugin::CommonRsp& rsp) {
  auto finditr = playback_action_map_ptr_->find(req.action_name());
  if (finditr == playback_action_map_ptr_->end()) {
    SetErrorCode(ErrorCode::kInvalidActionName, rsp);
    co_return aimrt::rpc::Status();
  }

  auto& action_wrapper = *(finditr->second);

  if (action_wrapper.GetOptions().mode != PlaybackAction::Options::Mode::kSignal) {
    SetErrorCode(ErrorCode::kInvalidActionMode, rsp);
    co_return aimrt::rpc::Status();
  }

  uint64_t skip_duration_s = req.skip_duration_s();
  uint64_t play_duration_s = req.play_duration_s();

  bool ret = action_wrapper.StartSignalPlayback(skip_duration_s, play_duration_s);
  if (!ret) {
    SetErrorCode(ErrorCode::kStartPlaybackFailed, rsp);
    co_return aimrt::rpc::Status();
  }

  co_return aimrt::rpc::Status();
}

aimrt::co::Task<aimrt::rpc::Status> RecordPlaybackServiceImpl::StopPlayback(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::record_playback_plugin::StopPlaybackReq& req,
    ::aimrt::protocols::record_playback_plugin::CommonRsp& rsp) {
  auto finditr = playback_action_map_ptr_->find(req.action_name());
  if (finditr == playback_action_map_ptr_->end()) {
    SetErrorCode(ErrorCode::kInvalidActionName, rsp);
    co_return aimrt::rpc::Status();
  }

  auto& action_wrapper = *(finditr->second);

  if (action_wrapper.GetOptions().mode != PlaybackAction::Options::Mode::kSignal) {
    SetErrorCode(ErrorCode::kInvalidActionMode, rsp);
    co_return aimrt::rpc::Status();
  }

  action_wrapper.StopSignalPlayback();

  co_return aimrt::rpc::Status();
}

aimrt::co::Task<aimrt::rpc::Status> RecordPlaybackServiceImpl::UpdateMetadata(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::record_playback_plugin::UpdateMetadataReq& req,
    ::aimrt::protocols::record_playback_plugin::CommonRsp& rsp) {
  auto finditr = record_action_map_ptr_->find(req.action_name());
  if (finditr == record_action_map_ptr_->end()) {
    SetErrorCode(ErrorCode::kInvalidActionName, rsp);
    co_return aimrt::rpc::Status();
  }
  std::unordered_map<std::string, std::string> kv_pairs;
  for (const auto& [key, value] : req.kv_pairs()) {
    if (key.empty()) [[unlikely]] {
      AIMRT_WARN("Received metadata update with empty key. Skipping.");
      continue;
    }
    kv_pairs[key] = value;
  }

  auto& action_wrapper = *(finditr->second);

  action_wrapper.UpdateMetadata(std::move(kv_pairs));
  co_return aimrt::rpc::Status();
}

}  // namespace aimrt::plugins::record_playback_plugin
