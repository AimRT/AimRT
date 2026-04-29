// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "agi_header_plugin/agi_header_plugin.h"

#include <charconv>
#include <string_view>

#include "aimrt_module_c_interface/channel/channel_context_base.h"
#include "aimrt_module_c_interface/rpc/rpc_context_base.h"
#include "core/aimrt_core.h"
#include "util/time_util.h"

namespace aimrt::plugins::agi_header_plugin {

namespace {
uint32_t ParseUint32OrDefault(std::string_view value, uint32_t default_val) {
  uint32_t result = default_val;
  if (!value.empty() &&
      std::from_chars(value.data(), value.data() + value.size(), result).ec == std::errc()) {
    return result;
  }
  return default_val;
}

uint64_t ParseUint64OrDefault(std::string_view value, uint64_t default_val) {
  uint64_t result = default_val;
  if (!value.empty() &&
      std::from_chars(value.data(), value.data() + value.size(), result).ec == std::errc()) {
    return result;
  }
  return default_val;
}

}  // namespace

bool AgiHeaderPlugin::Initialize(runtime::core::AimRTCore* core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;
    init_flag_ = true;

    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitChannel,
                                [this] { RegisterChannelFilter(); });
    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreInitRpc,
                                [this] { RegisterRpcFilter(); });
    core_ptr_->RegisterHookFunc(runtime::core::AimRTCore::State::kPreStart,
                                [this] {
                                  ScanRegisteredPublishTypes();
                                  ScanRegisteredRpcClientFuncs();
                                });

    return true;
  } catch (const std::exception&) {
  }

  return false;
}

void AgiHeaderPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;
    publish_header_info_map_.clear();
    rpc_request_header_info_map_.clear();
  } catch (const std::exception&) {
  }
}

void AgiHeaderPlugin::RegisterChannelFilter() {
  auto& channel_manager = core_ptr_->GetChannelManager();
  channel_manager.RegisterPublishFilter(
      "agi_header_fill",
      [this](aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
             aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
        PublishHeaderFilter(msg_wrapper, std::move(h));
      });
}

void AgiHeaderPlugin::RegisterRpcFilter() {
  auto& rpc_manager = core_ptr_->GetRpcManager();
  rpc_manager.RegisterClientFilter(
      "agi_request_header_fill",
      [this](const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
             aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
        RpcClientHeaderFilter(wrapper_ptr, std::move(h));
      });
}

void AgiHeaderPlugin::PublishHeaderFilter(
    aimrt::runtime::core::channel::MsgWrapper& msg_wrapper,
    aimrt::runtime::core::channel::FrameworkAsyncChannelHandle&& h) {
  const runtime::core::util::AgiHeaderInfo* info_ptr = nullptr;
  auto find_itr = publish_header_info_map_.find(PublishInfoKey{
      .msg_type = msg_wrapper.info.msg_type,
      .topic_name = msg_wrapper.info.topic_name,
      .pkg_path = msg_wrapper.info.pkg_path,
      .module_name = msg_wrapper.info.module_name,
  });
  if (find_itr != publish_header_info_map_.end())
    info_ptr = &(find_itr->second);

  if (info_ptr != nullptr && info_ptr->has_header) {
    auto ctx_ref = msg_wrapper.ctx_ref;
    uint32_t seq = ParseUint32OrDefault(
        ctx_ref.GetMetaValue(AIMRT_CHANNEL_CONTEXT_KEY_PUB_SEQ), 0);
    uint64_t timestamp_ns = ParseUint64OrDefault(
        ctx_ref.GetMetaValue(AIMRT_CHANNEL_CONTEXT_KEY_PUB_TIMESTAMP),
        aimrt::common::util::GetCurTimestampNs());

    runtime::core::util::FillAgiHeader(
        *info_ptr,
        const_cast<void*>(msg_wrapper.msg_ptr),
        seq,
        msg_wrapper.info.module_name,
        timestamp_ns);
  }

  h(msg_wrapper);
}

void AgiHeaderPlugin::RpcClientHeaderFilter(
    const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper>& wrapper_ptr,
    aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle&& h) {
  const runtime::core::util::AgiHeaderInfo* info_ptr = nullptr;
  auto find_itr = rpc_request_header_info_map_.find(RpcClientInfoKey{
      .func_name = wrapper_ptr->info.func_name,
      .pkg_path = wrapper_ptr->info.pkg_path,
      .module_name = wrapper_ptr->info.module_name,
  });
  if (find_itr != rpc_request_header_info_map_.end())
    info_ptr = &(find_itr->second);

  if (info_ptr != nullptr && info_ptr->has_header) {
    auto ctx_ref = wrapper_ptr->ctx_ref;
    uint32_t request_id = ParseUint32OrDefault(
        ctx_ref.GetMetaValue(AIMRT_RPC_CONTEXT_KEY_REQUEST_ID), 0);

    runtime::core::util::FillAgiRequestHeader(
        *info_ptr,
        const_cast<void*>(wrapper_ptr->req_ptr),
        request_id,
        wrapper_ptr->info.module_name,
        aimrt::common::util::GetCurTimestampNs());
  }

  h(wrapper_ptr);
}

void AgiHeaderPlugin::ScanRegisteredPublishTypes() {
  publish_header_info_map_.clear();
  const auto* registry_ptr = core_ptr_->GetChannelManager().GetChannelRegistry();
  if (registry_ptr == nullptr) return;

  const auto& wrapper_map = registry_ptr->GetPublishTypeWrapperMap();
  for (const auto& item : wrapper_map) {
    const auto& wrapper = *(item.second);
    const auto& info = wrapper.info;
    auto emplace_ret = publish_header_info_map_.emplace(
        PublishInfoKey{
            .msg_type = info.msg_type,
            .topic_name = info.topic_name,
            .pkg_path = info.pkg_path,
            .module_name = info.module_name,
        },
        runtime::core::util::DetectAgiHeader(
            info.msg_type,
            info.msg_type_support_ref.CustomTypeSupportPtr()));
    if (!emplace_ret.second) {
      emplace_ret.first->second = runtime::core::util::DetectAgiHeader(
          info.msg_type,
          info.msg_type_support_ref.CustomTypeSupportPtr());
    }
  }
}

void AgiHeaderPlugin::ScanRegisteredRpcClientFuncs() {
  rpc_request_header_info_map_.clear();
  const auto* registry_ptr = core_ptr_->GetRpcManager().GetRpcRegistry();
  if (registry_ptr == nullptr) return;

  const auto& wrapper_map = registry_ptr->GetClientFuncWrapperMap();
  for (const auto& item : wrapper_map) {
    const auto& wrapper = *(item.second);
    const auto& info = wrapper.info;
    auto emplace_ret = rpc_request_header_info_map_.emplace(
        RpcClientInfoKey{
            .func_name = info.func_name,
            .pkg_path = info.pkg_path,
            .module_name = info.module_name,
        },
        runtime::core::util::DetectAgiRequestHeader(
            info.req_type_support_ref.TypeName(),
            info.req_type_support_ref.CustomTypeSupportPtr()));
    if (!emplace_ret.second) {
      emplace_ret.first->second = runtime::core::util::DetectAgiRequestHeader(
          info.req_type_support_ref.TypeName(),
          info.req_type_support_ref.CustomTypeSupportPtr());
    }
  }
}

}  // namespace aimrt::plugins::agi_header_plugin
