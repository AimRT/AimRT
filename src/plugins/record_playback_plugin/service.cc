// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "record_playback_plugin/service.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "co/task.h"
#include "record_playback_plugin/global.h"
#include "record_playback_plugin/topic_meta.h"

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

aimrt::co::Task<aimrt::rpc::Status> RecordPlaybackServiceImpl::UpdateRecordAction(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::record_playback_plugin::UpdateRecordActionReq& req,
    ::aimrt::protocols::record_playback_plugin::UpdateRecordActionRsp& rsp) {
  AIMRT_INFO("Received update record action request: {}", req.DebugString());
  auto finditr = record_action_map_ptr_->find(req.action_name());
  auto common_rsp = rsp.mutable_common_rsp();
  if (finditr == record_action_map_ptr_->end()) {
    SetErrorCode(ErrorCode::kInvalidActionName, *common_rsp);
    co_return aimrt::rpc::Status();
  }

  auto& action_wrapper = *(finditr->second);
  std::vector<TopicMeta> topic_metas;
  for (const auto& topic_meta : req.topic_metas()) {
    topic_metas.emplace_back(TopicMeta{
        .topic_name = topic_meta.topic_name(),
        .msg_type = topic_meta.msg_type(),
        .record_enabled = topic_meta.record_enabled(),
        .sample_freq = topic_meta.has_sample_freq() ? topic_meta.sample_freq() : 0 });
  }

  action_wrapper.UpdateTopicMetaRecord(std::move(topic_metas), req.has_record_enabled() ? std::make_optional(req.record_enabled()) : std::nullopt);
  SetErrorCode(ErrorCode::kSuc, *common_rsp);
  co_return aimrt::rpc::Status();
}

aimrt::co::Task<aimrt::rpc::Status> RecordPlaybackServiceImpl::GetRecordActionStatus(
    aimrt::rpc::ContextRef ctx_ref,
    const ::aimrt::protocols::record_playback_plugin::GetRecordActionStatusReq& req,
    ::aimrt::protocols::record_playback_plugin::GetRecordActionStatusRsp& rsp) {
  auto common_rsp = rsp.mutable_common_rsp();

  std::vector<std::pair<std::string_view, RecordAction*>> action_vec;

  // If action_names is empty, then return all record actions' status.
  if (req.action_names().empty()) {
    action_vec.reserve(record_action_map_ptr_->size());
    for (auto& itr : *record_action_map_ptr_) {
      action_vec.emplace_back(itr.first, itr.second.get());
    }

    std::sort(
        action_vec.begin(), action_vec.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

  } else {
    action_vec.reserve(req.action_names_size());
    for (const auto& action_name : req.action_names()) {
      auto finditr = record_action_map_ptr_->find(action_name);
      if (finditr == record_action_map_ptr_->end()) {
        SetErrorCode(ErrorCode::kInvalidActionName, *common_rsp);
        co_return aimrt::rpc::Status();
      }

      action_vec.emplace_back(std::string_view(action_name), finditr->second.get());
    }
  }

  rsp.mutable_record_action_status_list()->Reserve(action_vec.size());

  std::vector<RecordAction::StatusSnapshot> snapshot_vec(action_vec.size());

  aimrt::util::DynamicLatch latch;
  for (size_t i = 0; i < action_vec.size(); ++i) {
    action_vec[i].second->GetStatusSnapshot(snapshot_vec[i], latch);
  }
  latch.CloseAndWait();

  for (size_t i = 0; i < action_vec.size(); ++i) {
    const auto& action_name = action_vec[i].first;
    const auto& snapshot = snapshot_vec[i];

    auto* pb_status = rsp.add_record_action_status_list();
    pb_status->set_action_name(std::string(action_name));
    pb_status->set_record_enabled(snapshot.record_enabled);

    pb_status->mutable_topic_meta_list()->Reserve(static_cast<int>(snapshot.topic_meta_list.size()));
    for (const auto& topic_meta : snapshot.topic_meta_list) {
      auto* pb_topic_meta = pb_status->add_topic_meta_list();
      pb_topic_meta->set_topic_name(topic_meta.topic_name);
      pb_topic_meta->set_msg_type(topic_meta.msg_type);
      pb_topic_meta->set_record_enabled(topic_meta.record_enabled);
      pb_topic_meta->set_sample_freq(topic_meta.sample_freq);
    }
  }

  SetErrorCode(ErrorCode::kSuc, *common_rsp);
  co_return aimrt::rpc::Status();
}

}  // namespace aimrt::plugins::record_playback_plugin
