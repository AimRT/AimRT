// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "iceoryx_plugin/util.h"

namespace aimrt::plugins::iceoryx_plugin {

// iceoryx rules each part of the service name should be less than 100 characters :IdString_t = cxx::string<100>;
std::string TruncateString(const std::string& input) {
  if (input.length() <= iox::MAX_RUNTIME_NAME_LENGTH) [[likely]] {
    return input;
  }

  size_t ellipsis_length = 3;
  size_t prefix_length = (iox::MAX_RUNTIME_NAME_LENGTH - ellipsis_length) / 2;
  size_t suffix_length = iox::MAX_RUNTIME_NAME_LENGTH - ellipsis_length - prefix_length;

  std::string truncated_string = input.substr(0, prefix_length) + "..." + input.substr(input.length() - suffix_length);
  AIMRT_WARN("Input url is too long. Each part should be less than {} characters. The input :{} has been truncated to :{}, which may lead to potential risks.", iox::MAX_RUNTIME_NAME_LENGTH, input, truncated_string);

  return truncated_string;
}

// iceoryx uses the iox::cxx::TruncateToCapacity_t to limit the length of the string to 100 characters
IdString_t String2IdString(const std::string& str) {
  std::string truncated_str = TruncateString(str);

  iox::cxx::TruncateToCapacity_t truncate_to_capacity;

  return {truncate_to_capacity, truncated_str.c_str(), truncated_str.length()};
}

// The iox description has three parts: "service name", "instance", "specific object"
iox::capro::ServiceDescription Url2ServiceDescription(std::string& url) {
  size_t first_slash_pos = url.find('/');
  size_t second_slash_pos = url.find('/', first_slash_pos + 1);
  size_t third_slash_pos = url.find('/', second_slash_pos + 1);

  return iox::capro::ServiceDescription{String2IdString(url.substr(0, second_slash_pos - 0)),
                                        String2IdString(url.substr(second_slash_pos, third_slash_pos - second_slash_pos)),
                                        String2IdString(url.substr(third_slash_pos))};
}

// Use PID as the unique name for each application/process
std::string GetPid() {
  std::string process_id_str;
#if defined(_WIN32)
  HANDLE hprocess = GetCurrentProcess();
  DWORD dwprocess_id = GetProcessId(hprocess);
  process_id_str = std::to_string(dwprocess_id);
#else
  pid_t pid = getpid();
  process_id_str = std::to_string(pid);
#endif
  return process_id_str;
}

std::string IntToFixedLengthString(int number, int length) {
  std::ostringstream oss;
  oss << std::setw(length) << number;
  return oss.str();
}

std::pair<std::shared_ptr<aimrt::util::BufferArrayView>, size_t> SerializeMsgSupportedIceoryx(
    MsgWrapper& msg_wrapper, std::string_view serialization_type, aimrt::util::BufferArrayAllocatorRef allocator) {
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

}  // namespace aimrt::plugins::iceoryx_plugin