// Copyright(c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "core/channel/channel_msg_wrapper.h"
#include "core/rpc/rpc_invoke_wrapper.h"

namespace aimrt::plugins::zenoh_plugin {

inline std::pair<std::shared_ptr<aimrt::util::BufferArrayView>, size_t> SerializeMsgSupportedZenoh(
    runtime::core::channel::MsgWrapper& msg_wrapper, std::string_view serialization_type, aimrt::util::BufferArrayAllocatorRef allocator) {
  auto& serialization_cache = msg_wrapper.serialization_cache;
  const auto& info = msg_wrapper.info;

  auto finditr = serialization_cache.find(serialization_type);
  if (finditr != serialization_cache.end()) {
    return {finditr->second, finditr->second->BufferSize()};
  }
  auto buffer_array_ptr = std::make_unique<aimrt::util::BufferArray>(allocator);
  bool serialize_ret = info.msg_type_support_ref.Serialize(
      serialization_type,
      msg_wrapper.msg_ptr,
      buffer_array_ptr->AllocatorNativeHandle(),
      buffer_array_ptr->BufferArrayNativeHandle());
  AIMRT_ASSERT(serialize_ret, "Serialize failed.");

  return {nullptr, buffer_array_ptr->BufferSize()};
}

inline std::pair<std::shared_ptr<aimrt::util::BufferArrayView>, size_t> SerializeReqSupportedZenoh(
    runtime::core::rpc::InvokeWrapper& invoke_wrapper, std::string_view serialization_type, aimrt::util::BufferArrayAllocatorRef allocator) {
  const auto& info = invoke_wrapper.info;
  auto& req_serialization_cache = invoke_wrapper.req_serialization_cache;

  auto find_itr = req_serialization_cache.find(serialization_type);
  if (find_itr != req_serialization_cache.end())
    return {find_itr->second, find_itr->second->BufferSize()};

  auto buffer_array_ptr = std::make_unique<aimrt::util::BufferArray>(allocator);
  bool serialize_ret = info.req_type_support_ref.Serialize(
      serialization_type,
      invoke_wrapper.req_ptr,
      buffer_array_ptr->AllocatorNativeHandle(),
      buffer_array_ptr->BufferArrayNativeHandle());

  AIMRT_ASSERT(serialize_ret, "Serialize failed.");

  return {nullptr, buffer_array_ptr->BufferSize()};
}

inline std::pair<std::shared_ptr<aimrt::util::BufferArrayView>, size_t> SerializeRspSupportedZenoh(
    runtime::core::rpc::InvokeWrapper& invoke_wrapper, std::string_view serialization_type, aimrt::util::BufferArrayAllocatorRef allocator) {
  const auto& info = invoke_wrapper.info;
  auto& rsp_serialization_cache = invoke_wrapper.rsp_serialization_cache;

  auto find_itr = rsp_serialization_cache.find(serialization_type);
  if (find_itr != rsp_serialization_cache.end())
    return {find_itr->second, find_itr->second->BufferSize()};

  auto buffer_array_ptr = std::make_unique<aimrt::util::BufferArray>(allocator);
  bool serialize_ret = info.rsp_type_support_ref.Serialize(
      serialization_type,
      invoke_wrapper.rsp_ptr,
      buffer_array_ptr->AllocatorNativeHandle(),
      buffer_array_ptr->BufferArrayNativeHandle());

  AIMRT_ASSERT(serialize_ret, "Serialize failed.");

  return {nullptr, buffer_array_ptr->BufferSize()};
}

}  // namespace aimrt::plugins::zenoh_plugin