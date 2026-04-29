// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>
#include <string_view>

namespace aimrt::runtime::core::util {

static constexpr std::string_view kAgiHeaderTypeName = "AgiHeader";
static constexpr std::string_view kAgiRequestHeaderTypeName = "AgiRequestHeader";

struct AgiTime {
  int32_t seconds = 0;
  uint32_t fraction = 0;
};

inline AgiTime NanosToAgiTime(uint64_t timestamp_ns) {
  AgiTime t;
  t.seconds = static_cast<int32_t>(timestamp_ns / 1'000'000'000ULL);
  uint64_t nanos = timestamp_ns % 1'000'000'000ULL;
  t.fraction = static_cast<uint32_t>((nanos << 32) / 1'000'000'000ULL);
  return t;
}

enum class AgiHeaderType {
  kNone,
  kProtobuf,
  kRos2,
};

struct AgiHeaderInfo {
  bool has_header = false;
  AgiHeaderType type = AgiHeaderType::kNone;

  // For protobuf: stores the FieldDescriptor* of the AgiHeader/AgiRequestHeader field
  const void* pb_header_field = nullptr;

  // For ros2: stores the offset of the AgiHeader sub-message within the parent
  size_t ros2_header_offset = 0;
  // Stores the MessageMembers* of the AgiHeader sub-message for field-level access
  const void* ros2_header_members = nullptr;
};

AgiHeaderInfo DetectAgiHeader(std::string_view msg_type, const void* custom_type_support_ptr);
AgiHeaderInfo DetectAgiRequestHeader(std::string_view msg_type, const void* custom_type_support_ptr);

void FillAgiHeader(const AgiHeaderInfo& info, void* msg_ptr,
                   uint32_t seq_num, std::string_view publisher_name,
                   uint64_t publish_time_ns);

void FillAgiRequestHeader(const AgiHeaderInfo& info, void* msg_ptr,
                          uint32_t request_id, std::string_view client_name,
                          uint64_t request_time_ns);

}  // namespace aimrt::runtime::core::util
