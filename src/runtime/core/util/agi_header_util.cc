// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/util/agi_header_util.h"

#ifdef AIMRT_BUILD_WITH_PROTOBUF
  #include <google/protobuf/descriptor.h>
  #include <google/protobuf/message.h>
#endif

#ifdef AIMRT_BUILD_WITH_ROS2
  #include "rosidl_typesupport_cpp/message_type_support.hpp"
  #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
  #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
  #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#endif

namespace aimrt::runtime::core::util {

namespace {

#ifdef AIMRT_BUILD_WITH_PROTOBUF

const google::protobuf::FieldDescriptor* FindPbSubMessageField(
    const void* custom_type_support_ptr, std::string_view target_type_name) {
  if (!custom_type_support_ptr) return nullptr;

  const auto* descriptor =
      static_cast<const google::protobuf::Descriptor*>(custom_type_support_ptr);

  for (int i = 0; i < descriptor->field_count(); i++) {
    const auto* field = descriptor->field(i);
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE &&
        field->message_type()->name() == target_type_name) {
      return field;
    }
  }
  return nullptr;
}

void FillPbTimeField(google::protobuf::Message* header,
                     const google::protobuf::FieldDescriptor* time_field,
                     uint64_t timestamp_ns) {
  if (!time_field) return;

  const auto* reflection = header->GetReflection();
  auto* time_msg = reflection->MutableMessage(header, time_field);
  const auto* time_desc = time_msg->GetDescriptor();
  const auto* time_refl = time_msg->GetReflection();

  AgiTime t = NanosToAgiTime(timestamp_ns);

  const auto* seconds_field = time_desc->FindFieldByName("seconds");
  const auto* fraction_field = time_desc->FindFieldByName("fraction");
  if (seconds_field) time_refl->SetInt32(time_msg, seconds_field, t.seconds);
  if (fraction_field) time_refl->SetUInt32(time_msg, fraction_field, t.fraction);
}

#endif  // AIMRT_BUILD_WITH_PROTOBUF

#ifdef AIMRT_BUILD_WITH_ROS2

const rosidl_message_type_support_t* ImproveToIntrospectionTypeSupport(
    const rosidl_message_type_support_t* type_supports) {
  return get_message_typesupport_handle(
      type_supports,
      rosidl_typesupport_introspection_cpp::typesupport_identifier);
}

const rosidl_typesupport_introspection_cpp::MessageMembers* GetRosMembersInfo(
    const rosidl_message_type_support_t* ts) {
  return reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
      ImproveToIntrospectionTypeSupport(ts)->data);
}

struct Ros2FieldResult {
  bool found = false;
  size_t offset = 0;
  const rosidl_typesupport_introspection_cpp::MessageMembers* sub_members = nullptr;
};

Ros2FieldResult FindRos2SubMessageField(
    const void* custom_type_support_ptr, std::string_view target_type_name) {
  if (!custom_type_support_ptr) return {};

  const auto* ts =
      static_cast<const rosidl_message_type_support_t*>(custom_type_support_ptr);

  const auto* members = GetRosMembersInfo(ts);
  if (!members) return {};

  for (uint32_t i = 0; i < members->member_count_; i++) {
    const auto& member = members->members_[i];
    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE &&
        member.members_ != nullptr) {
      const auto* sub_members =
          static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
              member.members_->data);
      if (sub_members && std::string_view(sub_members->message_name_) == target_type_name) {
        return {true, member.offset_, sub_members};
      }
    }
  }
  return {};
}

size_t FindRos2MemberOffset(
    const rosidl_typesupport_introspection_cpp::MessageMembers* members,
    std::string_view field_name) {
  for (uint32_t i = 0; i < members->member_count_; i++) {
    if (std::string_view(members->members_[i].name_) == field_name) {
      return members->members_[i].offset_;
    }
  }
  return static_cast<size_t>(-1);
}

const rosidl_typesupport_introspection_cpp::MessageMembers* FindRos2SubMemberMembers(
    const rosidl_typesupport_introspection_cpp::MessageMembers* members,
    std::string_view field_name) {
  for (uint32_t i = 0; i < members->member_count_; i++) {
    if (std::string_view(members->members_[i].name_) == field_name &&
        members->members_[i].type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE &&
        members->members_[i].members_ != nullptr) {
      return static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
          members->members_[i].members_->data);
    }
  }
  return nullptr;
}

void FillRos2TimeField(uint8_t* time_ptr,
                       const rosidl_typesupport_introspection_cpp::MessageMembers* time_members,
                       uint64_t timestamp_ns) {
  if (!time_members) return;
  AgiTime t = NanosToAgiTime(timestamp_ns);

  size_t seconds_offset = FindRos2MemberOffset(time_members, "seconds");
  size_t fraction_offset = FindRos2MemberOffset(time_members, "fraction");

  if (seconds_offset != static_cast<size_t>(-1))
    *reinterpret_cast<int32_t*>(time_ptr + seconds_offset) = t.seconds;
  if (fraction_offset != static_cast<size_t>(-1))
    *reinterpret_cast<uint32_t*>(time_ptr + fraction_offset) = t.fraction;
}

#endif  // AIMRT_BUILD_WITH_ROS2

}  // namespace

AgiHeaderInfo DetectAgiHeader(std::string_view msg_type, const void* custom_type_support_ptr) {
  AgiHeaderInfo info;

#ifdef AIMRT_BUILD_WITH_PROTOBUF
  if (msg_type.substr(0, 3) == "pb:") {
    const auto* field = FindPbSubMessageField(custom_type_support_ptr, kAgiHeaderTypeName);
    if (field) {
      info.has_header = true;
      info.type = AgiHeaderType::kProtobuf;
      info.pb_header_field = field;
      return info;
    }
  }
#endif

#ifdef AIMRT_BUILD_WITH_ROS2
  if (msg_type.substr(0, 5) == "ros2:") {
    auto result = FindRos2SubMessageField(custom_type_support_ptr, kAgiHeaderTypeName);
    if (result.found) {
      info.has_header = true;
      info.type = AgiHeaderType::kRos2;
      info.ros2_header_offset = result.offset;
      info.ros2_header_members = result.sub_members;
      return info;
    }
  }
#endif

  return info;
}

AgiHeaderInfo DetectAgiRequestHeader(std::string_view msg_type, const void* custom_type_support_ptr) {
  AgiHeaderInfo info;

#ifdef AIMRT_BUILD_WITH_PROTOBUF
  if (msg_type.substr(0, 3) == "pb:") {
    const auto* field = FindPbSubMessageField(custom_type_support_ptr, kAgiRequestHeaderTypeName);
    if (field) {
      info.has_header = true;
      info.type = AgiHeaderType::kProtobuf;
      info.pb_header_field = field;
      return info;
    }
  }
#endif

#ifdef AIMRT_BUILD_WITH_ROS2
  if (msg_type.substr(0, 5) == "ros2:") {
    auto result = FindRos2SubMessageField(custom_type_support_ptr, kAgiRequestHeaderTypeName);
    if (result.found) {
      info.has_header = true;
      info.type = AgiHeaderType::kRos2;
      info.ros2_header_offset = result.offset;
      info.ros2_header_members = result.sub_members;
      return info;
    }
  }
#endif

  return info;
}

void FillAgiHeader(const AgiHeaderInfo& info, void* msg_ptr,
                   uint32_t seq_num, std::string_view publisher_name,
                   uint64_t publish_time_ns) {
  if (!info.has_header || !msg_ptr) return;

#ifdef AIMRT_BUILD_WITH_PROTOBUF
  if (info.type == AgiHeaderType::kProtobuf) {
    auto* msg = static_cast<google::protobuf::Message*>(msg_ptr);
    const auto* field =
        static_cast<const google::protobuf::FieldDescriptor*>(info.pb_header_field);
    const auto* reflection = msg->GetReflection();
    auto* header = reflection->MutableMessage(msg, field);
    const auto* header_desc = header->GetDescriptor();
    const auto* header_refl = header->GetReflection();

    const auto* seq_field = header_desc->FindFieldByName("seq_num");
    const auto* name_field = header_desc->FindFieldByName("publisher_name");
    const auto* time_field = header_desc->FindFieldByName("publish_time");

    if (seq_field) header_refl->SetUInt32(header, seq_field, seq_num);
    if (name_field) header_refl->SetString(header, name_field, std::string(publisher_name));
    if (time_field) FillPbTimeField(header, time_field, publish_time_ns);
    return;
  }
#endif

#ifdef AIMRT_BUILD_WITH_ROS2
  if (info.type == AgiHeaderType::kRos2) {
    auto* msg_bytes = static_cast<uint8_t*>(msg_ptr);
    auto* header_ptr = msg_bytes + info.ros2_header_offset;
    const auto* members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
            info.ros2_header_members);
    if (!members) return;

    size_t seq_offset = FindRos2MemberOffset(members, "seq_num");
    size_t name_offset = FindRos2MemberOffset(members, "publisher_name");

    if (seq_offset != static_cast<size_t>(-1))
      *reinterpret_cast<uint32_t*>(header_ptr + seq_offset) = seq_num;
    if (name_offset != static_cast<size_t>(-1))
      *reinterpret_cast<std::string*>(header_ptr + name_offset) = std::string(publisher_name);

    const auto* time_members = FindRos2SubMemberMembers(members, "publish_time");
    if (time_members) {
      size_t time_offset = FindRos2MemberOffset(members, "publish_time");
      if (time_offset != static_cast<size_t>(-1))
        FillRos2TimeField(header_ptr + time_offset, time_members, publish_time_ns);
    }
    return;
  }
#endif
}

void FillAgiRequestHeader(const AgiHeaderInfo& info, void* msg_ptr,
                          uint32_t request_id, std::string_view client_name,
                          uint64_t request_time_ns) {
  if (!info.has_header || !msg_ptr) return;

#ifdef AIMRT_BUILD_WITH_PROTOBUF
  if (info.type == AgiHeaderType::kProtobuf) {
    auto* msg = static_cast<google::protobuf::Message*>(msg_ptr);
    const auto* field =
        static_cast<const google::protobuf::FieldDescriptor*>(info.pb_header_field);
    const auto* reflection = msg->GetReflection();
    auto* header = reflection->MutableMessage(msg, field);
    const auto* header_desc = header->GetDescriptor();
    const auto* header_refl = header->GetReflection();

    const auto* id_field = header_desc->FindFieldByName("request_id");
    const auto* name_field = header_desc->FindFieldByName("client_name");
    const auto* time_field = header_desc->FindFieldByName("request_time");

    if (id_field) header_refl->SetUInt32(header, id_field, request_id);
    if (name_field) header_refl->SetString(header, name_field, std::string(client_name));
    if (time_field) FillPbTimeField(header, time_field, request_time_ns);
    return;
  }
#endif

#ifdef AIMRT_BUILD_WITH_ROS2
  if (info.type == AgiHeaderType::kRos2) {
    auto* msg_bytes = static_cast<uint8_t*>(msg_ptr);
    auto* header_ptr = msg_bytes + info.ros2_header_offset;
    const auto* members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
            info.ros2_header_members);
    if (!members) return;

    size_t id_offset = FindRos2MemberOffset(members, "request_id");
    size_t name_offset = FindRos2MemberOffset(members, "client_name");

    if (id_offset != static_cast<size_t>(-1))
      *reinterpret_cast<uint32_t*>(header_ptr + id_offset) = request_id;
    if (name_offset != static_cast<size_t>(-1))
      *reinterpret_cast<std::string*>(header_ptr + name_offset) = std::string(client_name);

    const auto* time_members = FindRos2SubMemberMembers(members, "request_time");
    if (time_members) {
      size_t time_offset = FindRos2MemberOffset(members, "request_time");
      if (time_offset != static_cast<size_t>(-1))
        FillRos2TimeField(header_ptr + time_offset, time_members, request_time_ns);
    }
    return;
  }
#endif
}

}  // namespace aimrt::runtime::core::util
