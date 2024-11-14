// Copyright(c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/channel/channel_msg_wrapper.h"

namespace aimrt::plugins::zenoh_plugin {
constexpr unsigned int kFixedLen = 20;  // FIXED_LEN represents the length of the pkg_size's string， which is enough to the max value of uint64_t

inline std::pair<std::shared_ptr<aimrt::util::BufferArrayView>, size_t> SerializeMsgSupportedZenoh(
    runtime::core::channel::MsgWrapper& msg_wrapper, std::string_view serialization_type, aimrt::util::BufferArrayAllocatorRef allocator) {
  auto& serialization_cache = msg_wrapper.serialization_cache;
  const auto& info = msg_wrapper.info;

  size_t result = 0;
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

inline std::string IntToFixedLengthString(int number, int length) {
  std::ostringstream oss;
  oss << std::setw(length) << number;
  return oss.str();
}

}  // namespace aimrt::plugins::zenoh_plugin