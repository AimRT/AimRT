// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "core/rpc/rpc_invoke_wrapper.h"

namespace aimrt::runtime::core::rpc {

inline std::shared_ptr<aimrt::util::BufferArrayView> SerializeReqWithCache(
    InvokeWrapper& invoke_wrapper, std::string_view serialization_type) {
  const auto& info = invoke_wrapper.info;
  auto& req_serialization_cache = invoke_wrapper.req_serialization_cache;

  auto find_itr = req_serialization_cache.find(serialization_type);
  if (find_itr != req_serialization_cache.end())
    return find_itr->second;

  auto buffer_array_ptr = std::make_unique<aimrt::util::BufferArray>();
  bool serialize_ret = info.req_type_support_ref.Serialize(
      serialization_type,
      invoke_wrapper.req_ptr,
      buffer_array_ptr->AllocatorNativeHandle(),
      buffer_array_ptr->BufferArrayNativeHandle());

  AIMRT_ASSERT(serialize_ret, "Serialize failed.");

  auto* ptr = buffer_array_ptr.get();
  auto buffer_array_view_ptr = std::shared_ptr<aimrt::util::BufferArrayView>(
      new aimrt::util::BufferArrayView(*ptr),
      [buffer_array_ptr{std::move(buffer_array_ptr)}](const auto* ptr) { delete ptr; });

  req_serialization_cache.emplace(serialization_type, buffer_array_view_ptr);

  return buffer_array_view_ptr;
}

inline std::shared_ptr<aimrt::util::BufferArrayView> TrySerializeReqWithCache(
    InvokeWrapper& invoke_wrapper, std::string_view serialization_type) noexcept {
  try {
    return SerializeReqWithCache(invoke_wrapper, serialization_type);
  } catch (...) {
    return {};
  }
}

inline std::shared_ptr<aimrt::util::BufferArrayView> SerializeRspWithCache(
    InvokeWrapper& invoke_wrapper, std::string_view serialization_type) {
  const auto& info = invoke_wrapper.info;
  auto& rsp_serialization_cache = invoke_wrapper.rsp_serialization_cache;

  auto find_itr = rsp_serialization_cache.find(serialization_type);
  if (find_itr != rsp_serialization_cache.end())
    return find_itr->second;

  auto buffer_array_ptr = std::make_unique<aimrt::util::BufferArray>();
  bool serialize_ret = info.rsp_type_support_ref.Serialize(
      serialization_type,
      invoke_wrapper.rsp_ptr,
      buffer_array_ptr->AllocatorNativeHandle(),
      buffer_array_ptr->BufferArrayNativeHandle());

  AIMRT_ASSERT(serialize_ret, "Serialize failed.");

  auto* ptr = buffer_array_ptr.get();
  auto buffer_array_view_ptr = std::shared_ptr<aimrt::util::BufferArrayView>(
      new aimrt::util::BufferArrayView(*ptr),
      [buffer_array_ptr{std::move(buffer_array_ptr)}](const auto* ptr) { delete ptr; });

  rsp_serialization_cache.emplace(serialization_type, buffer_array_view_ptr);

  return buffer_array_view_ptr;
}

inline std::shared_ptr<aimrt::util::BufferArrayView> TrySerializeRspWithCache(
    InvokeWrapper& invoke_wrapper, std::string_view serialization_type) noexcept {
  try {
    return SerializeRspWithCache(invoke_wrapper, serialization_type);
  } catch (...) {
    return {};
  }
}

inline void InvokeCallBack(InvokeWrapper& invoke_wrapper, aimrt::rpc::Status status) {
  invoke_wrapper.callback(status);
  invoke_wrapper.callback = nullptr;
}

}  // namespace aimrt::runtime::core::rpc
