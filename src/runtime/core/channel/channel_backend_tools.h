// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <vector>

#include "core/channel/channel_registry.h"

namespace aimrt::runtime::core::channel {

class SubscribeTool {
 public:
  SubscribeTool() = default;
  ~SubscribeTool() = default;

  SubscribeTool(const SubscribeTool&) = delete;
  SubscribeTool& operator=(const SubscribeTool&) = delete;

  void AddSubscribeWrapper(const runtime::core::channel::SubscribeWrapper* sub_wrapper_ptr) {
    require_cache_serialization_types_.insert(
        sub_wrapper_ptr->require_cache_serialization_types.begin(),
        sub_wrapper_ptr->require_cache_serialization_types.end());
    sub_wrapper_vec_.emplace_back(sub_wrapper_ptr);
  }

  void DoSubscribeCallback(
      const std::shared_ptr<aimrt::channel::Context>& ctx_ptr,
      const runtime::core::channel::SubscribeWrapper& msg_sub_wrapper,
      const std::shared_ptr<void>& msg_ptr) const {
    AIMRT_ASSERT(&msg_sub_wrapper == sub_wrapper_vec_[0], "Unexpected errors.");

    runtime::core::channel::MsgWrapper sub_msg_wrapper{
        .info = msg_sub_wrapper.info,
        .msg_ptr = msg_ptr.get(),
        .ctx_ref = ctx_ptr};

    sub_msg_wrapper.msg_cache_ptr = msg_ptr;

    msg_sub_wrapper.callback(sub_msg_wrapper, [ctx_ptr]() {});

    if (sub_wrapper_vec_.size() == 1) return;

    auto buffer_array_ptr = std::make_unique<aimrt::util::BufferArray>();
    auto serialization_type = msg_sub_wrapper.info.msg_type_support_ref.DefaultSerializationType();

    bool serialize_ret = msg_sub_wrapper.info.msg_type_support_ref.Serialize(
        serialization_type,
        msg_ptr.get(),
        buffer_array_ptr->AllocatorNativeHandle(),
        buffer_array_ptr->BufferArrayNativeHandle());

    AIMRT_ASSERT(serialize_ret, "Serialize failed.");

    auto* ptr = buffer_array_ptr.get();
    auto buffer_array_view_ptr = std::shared_ptr<aimrt::util::BufferArrayView>(
        new aimrt::util::BufferArrayView(*ptr),
        [buffer_array_ptr{std::move(buffer_array_ptr)}](const auto* ptr) { delete ptr; });

    for (size_t ii = 1; ii < sub_wrapper_vec_.size(); ++ii) {
      const auto* sub_wrapper_ptr = sub_wrapper_vec_[ii];
      runtime::core::channel::MsgWrapper sub_msg_wrapper{
          .info = sub_wrapper_ptr->info,
          .msg_ptr = nullptr,
          .ctx_ref = ctx_ptr};

      sub_msg_wrapper.serialization_cache.emplace(serialization_type, buffer_array_view_ptr);

      sub_wrapper_ptr->callback(sub_msg_wrapper, [ctx_ptr]() {});
    }
  }

  void DoSubscribeCallback(
      const std::shared_ptr<aimrt::channel::Context>& ctx_ptr,
      const std::string& serialization_type,
      const aimrt::util::BufferArrayView& buffer_array_view) const {
    bool need_cache_flag =
        (require_cache_serialization_types_.find(serialization_type) != require_cache_serialization_types_.end());

    if (!need_cache_flag) {
      DoSubscribeCallbackWithoutCache(ctx_ptr, serialization_type, buffer_array_view);
    } else {
      const auto* buffer_array_data = buffer_array_view.Data();
      const size_t buffer_array_len = buffer_array_view.Size();
      size_t msg_size = buffer_array_view.BufferSize();

      std::unique_ptr<std::vector<uint8_t>> buffer_ptr = std::make_unique<std::vector<uint8_t>>(msg_size);
      uint8_t* cur_pos = buffer_ptr->data();
      for (size_t ii = 0; ii < buffer_array_len; ++ii) {
        memcpy(cur_pos, buffer_array_data[ii].data, buffer_array_data[ii].len);
        cur_pos += buffer_array_data[ii].len;
      }

      auto* ptr = buffer_ptr.get();
      auto buffer_array_view_ptr = std::shared_ptr<aimrt::util::BufferArrayView>(
          new aimrt::util::BufferArrayView(ptr->data(), ptr->size()),
          [buffer_ptr{std::move(buffer_ptr)}](const auto* ptr) { delete ptr; });

      DoSubscribeCallbackWithCache(ctx_ptr, serialization_type, buffer_array_view_ptr);
    }
  }

  void DoSubscribeCallback(
      const std::shared_ptr<aimrt::channel::Context>& ctx_ptr,
      const std::string& serialization_type,
      const void* data, size_t len) const {
    bool need_cache_flag =
        (require_cache_serialization_types_.find(serialization_type) != require_cache_serialization_types_.end());

    if (!need_cache_flag) {
      aimrt::util::BufferArrayView buffer_array_view(data, len);
      DoSubscribeCallbackWithoutCache(ctx_ptr, serialization_type, buffer_array_view);
    } else {
      std::unique_ptr<std::vector<uint8_t>> buffer_ptr = std::make_unique<std::vector<uint8_t>>(len);
      memcpy(buffer_ptr->data(), data, len);

      auto* ptr = buffer_ptr.get();
      auto buffer_array_view_ptr = std::shared_ptr<aimrt::util::BufferArrayView>(
          new aimrt::util::BufferArrayView(ptr->data(), ptr->size()),
          [buffer_ptr{std::move(buffer_ptr)}](const auto* ptr) { delete ptr; });

      DoSubscribeCallbackWithCache(ctx_ptr, serialization_type, buffer_array_view_ptr);
    }
  }

 private:
  void DoSubscribeCallbackWithoutCache(
      const std::shared_ptr<aimrt::channel::Context>& ctx_ptr,
      const std::string& serialization_type,
      const aimrt::util::BufferArrayView& buffer_array_view) const {
    for (const auto* sub_wrapper_ptr : sub_wrapper_vec_) {
      auto subscribe_type_support_ref = sub_wrapper_ptr->info.msg_type_support_ref;

      std::shared_ptr<void> msg_ptr = subscribe_type_support_ref.CreateSharedPtr();

      bool deserialize_ret = subscribe_type_support_ref.Deserialize(
          serialization_type, *(buffer_array_view.NativeHandle()), msg_ptr.get());

      AIMRT_ASSERT(deserialize_ret, "Msg deserialize failed.");

      runtime::core::channel::MsgWrapper sub_msg_wrapper{
          .info = sub_wrapper_ptr->info,
          .msg_ptr = msg_ptr.get(),
          .ctx_ref = ctx_ptr};

      sub_msg_wrapper.msg_cache_ptr = msg_ptr;

      sub_wrapper_ptr->callback(sub_msg_wrapper, [ctx_ptr]() {});
    }
  }

  void DoSubscribeCallbackWithCache(
      const std::shared_ptr<aimrt::channel::Context>& ctx_ptr,
      const std::string& serialization_type,
      const std::shared_ptr<aimrt::util::BufferArrayView>& buffer_array_view_ptr) const {
    for (const auto* sub_wrapper_ptr : sub_wrapper_vec_) {
      runtime::core::channel::MsgWrapper sub_msg_wrapper{
          .info = sub_wrapper_ptr->info,
          .msg_ptr = nullptr,
          .ctx_ref = ctx_ptr};

      sub_msg_wrapper.serialization_cache.emplace(serialization_type, buffer_array_view_ptr);

      sub_wrapper_ptr->callback(sub_msg_wrapper, [ctx_ptr]() {});
    }
  }

 private:
  std::vector<const runtime::core::channel::SubscribeWrapper*> sub_wrapper_vec_;
  std::unordered_set<std::string> require_cache_serialization_types_;
};

inline void CheckMsg(MsgWrapper& msg_wrapper) {
  if (msg_wrapper.msg_ptr != nullptr) return;

  const auto& serialization_cache = msg_wrapper.serialization_cache;
  const auto& info = msg_wrapper.info;

  AIMRT_ASSERT(!serialization_cache.empty(),
               "Can not get msg, msg is null and serialization cache is empty.");

  if (serialization_cache.size() == 1) {
    auto msg_cache_ptr = info.msg_type_support_ref.CreateSharedPtr();

    const auto& serialization_type = serialization_cache.begin()->first;
    auto buffer_array_view_ptr = serialization_cache.begin()->second;

    bool deserialize_ret = info.msg_type_support_ref.Deserialize(
        serialization_type, *(buffer_array_view_ptr->NativeHandle()), msg_cache_ptr.get());

    AIMRT_ASSERT(deserialize_ret,
                 "Can not get msg, msg is null and deserialize failed.");

    msg_wrapper.msg_cache_ptr = std::move(msg_cache_ptr);
    msg_wrapper.msg_ptr = msg_wrapper.msg_cache_ptr.get();
    return;
  }

  auto serialization_type_span = info.msg_type_support_ref.SerializationTypesSupportedListSpan();

  for (auto item : serialization_type_span) {
    auto serialization_type = aimrt::util::ToStdStringView(item);

    auto finditr = serialization_cache.find(serialization_type);
    if (finditr == serialization_cache.end()) [[unlikely]]
      continue;

    auto msg_cache_ptr = info.msg_type_support_ref.CreateSharedPtr();

    auto buffer_array_view_ptr = finditr->second;

    bool deserialize_ret = info.msg_type_support_ref.Deserialize(
        serialization_type, *(buffer_array_view_ptr->NativeHandle()), msg_cache_ptr.get());

    AIMRT_ASSERT(deserialize_ret,
                 "Can not get msg, msg is null and deserialize failed.");

    msg_wrapper.msg_cache_ptr = std::move(msg_cache_ptr);
    msg_wrapper.msg_ptr = msg_wrapper.msg_cache_ptr.get();
    return;
  }

  throw aimrt::common::util::AimRTException("Can not get msg, msg is null and can not deserialize from cache.");
}

inline bool TryCheckMsg(MsgWrapper& msg_wrapper) noexcept {
  try {
    CheckMsg(msg_wrapper);
    return true;
  } catch (...) {
    return false;
  }
}

inline std::shared_ptr<aimrt::util::BufferArrayView> SerializeMsgWithCache(
    MsgWrapper& msg_wrapper, std::string_view serialization_type) {
  auto& serialization_cache = msg_wrapper.serialization_cache;
  const auto& info = msg_wrapper.info;

  auto finditr = serialization_cache.find(serialization_type);
  if (finditr != serialization_cache.end())
    return finditr->second;

  CheckMsg(msg_wrapper);

  auto buffer_array_ptr = std::make_unique<aimrt::util::BufferArray>();
  bool serialize_ret = info.msg_type_support_ref.Serialize(
      serialization_type,
      msg_wrapper.msg_ptr,
      buffer_array_ptr->AllocatorNativeHandle(),
      buffer_array_ptr->BufferArrayNativeHandle());

  AIMRT_ASSERT(serialize_ret, "Serialize failed.");

  auto* ptr = buffer_array_ptr.get();
  auto buffer_array_view_ptr = std::shared_ptr<aimrt::util::BufferArrayView>(
      new aimrt::util::BufferArrayView(*ptr),
      [buffer_array_ptr{std::move(buffer_array_ptr)}](const auto* ptr) { delete ptr; });

  serialization_cache.emplace(serialization_type, buffer_array_view_ptr);

  return buffer_array_view_ptr;
}

inline std::shared_ptr<aimrt::util::BufferArrayView> TrySerializeMsgWithCache(
    MsgWrapper& msg_wrapper, std::string_view serialization_type) noexcept {
  try {
    return SerializeMsgWithCache(msg_wrapper, serialization_type);
  } catch (...) {
    return {};
  }
}

}  // namespace aimrt::runtime::core::channel
