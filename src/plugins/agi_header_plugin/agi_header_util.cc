// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "agi_header_util.h"

#ifdef AIMRT_BUILD_WITH_PROTOBUF
  #include <google/protobuf/descriptor.h>
  #include <google/protobuf/message.h>
#endif

#ifdef AIMRT_BUILD_WITH_ROS2
  #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
  #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
  #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
  #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#endif

namespace aimrt::runtime::core::util {

namespace {

#ifdef AIMRT_BUILD_WITH_PROTOBUF

bool IsPbFieldType(const google::protobuf::Descriptor* descriptor,
                   std::string_view field_name,
                   google::protobuf::FieldDescriptor::Type field_type) {
  const auto* field = descriptor->FindFieldByName(std::string(field_name));
  return field != nullptr && !field->is_repeated() && field->type() == field_type;
}

bool IsPbTimeSchema(const google::protobuf::Descriptor* descriptor) {
  return descriptor != nullptr &&
         IsPbFieldType(descriptor, "seconds", google::protobuf::FieldDescriptor::TYPE_INT32) &&
         IsPbFieldType(descriptor, "fraction", google::protobuf::FieldDescriptor::TYPE_UINT32);
}

bool IsPbMessageFieldWithSchema(const google::protobuf::Descriptor* descriptor,
                                std::string_view field_name,
                                bool (*schema_checker)(const google::protobuf::Descriptor*)) {
  const auto* field = descriptor->FindFieldByName(std::string(field_name));
  return field != nullptr && !field->is_repeated() &&
         field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE &&
         schema_checker(field->message_type());
}

bool IsPbAgiHeaderSchema(const google::protobuf::Descriptor* descriptor) {
  return descriptor != nullptr && std::string_view(descriptor->name()) == kAgiHeaderTypeName &&
         IsPbFieldType(descriptor, "seq_num", google::protobuf::FieldDescriptor::TYPE_UINT32) &&
         IsPbFieldType(descriptor, "publisher_name", google::protobuf::FieldDescriptor::TYPE_STRING) &&
         IsPbMessageFieldWithSchema(descriptor, "publish_time", IsPbTimeSchema);
}

bool IsPbAgiRequestHeaderSchema(const google::protobuf::Descriptor* descriptor) {
  return descriptor != nullptr && std::string_view(descriptor->name()) == kAgiRequestHeaderTypeName &&
         IsPbFieldType(descriptor, "request_id", google::protobuf::FieldDescriptor::TYPE_UINT32) &&
         IsPbFieldType(descriptor, "client_name", google::protobuf::FieldDescriptor::TYPE_STRING) &&
         IsPbMessageFieldWithSchema(descriptor, "request_time", IsPbTimeSchema);
}

const google::protobuf::FieldDescriptor* FindPbSubMessageField(
    const void* custom_type_support_ptr,
    bool (*schema_checker)(const google::protobuf::Descriptor*)) {
  if (!custom_type_support_ptr) return nullptr;

  const auto* descriptor =
      static_cast<const google::protobuf::Descriptor*>(custom_type_support_ptr);

  for (int i = 0; i < descriptor->field_count(); i++) {
    const auto* field = descriptor->field(i);
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE &&
        !field->is_repeated() &&
        schema_checker(field->message_type())) {
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
  if (seconds_field && seconds_field->type() == google::protobuf::FieldDescriptor::TYPE_INT32)
    time_refl->SetInt32(time_msg, seconds_field, t.seconds);
  if (fraction_field && fraction_field->type() == google::protobuf::FieldDescriptor::TYPE_UINT32)
    time_refl->SetUInt32(time_msg, fraction_field, t.fraction);
}

#endif  // AIMRT_BUILD_WITH_PROTOBUF

#ifdef AIMRT_BUILD_WITH_ROS2

const rosidl_message_type_support_t* ImproveToIntrospectionTypeSupport(
    const rosidl_message_type_support_t* type_supports) {
  if (!type_supports) return nullptr;
  if (std::string_view(type_supports->typesupport_identifier) ==
      rosidl_typesupport_introspection_cpp::typesupport_identifier) {
    return type_supports;
  }
  return rosidl_typesupport_cpp::get_message_typesupport_handle_function(
      type_supports,
      rosidl_typesupport_introspection_cpp::typesupport_identifier);
}

const rosidl_typesupport_introspection_cpp::MessageMembers* GetRosMembersInfo(
    const rosidl_message_type_support_t* ts) {
  const auto* introspection_ts = ImproveToIntrospectionTypeSupport(ts);
  if (!introspection_ts || !introspection_ts->data) return nullptr;
  return reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
      introspection_ts->data);
}

struct Ros2FieldResult {
  bool found = false;
  size_t offset = 0;
  const rosidl_typesupport_introspection_cpp::MessageMembers* sub_members = nullptr;
};

const rosidl_typesupport_introspection_cpp::MessageMember* FindRos2Member(
    const rosidl_typesupport_introspection_cpp::MessageMembers* members,
    std::string_view field_name) {
  if (!members) return nullptr;
  for (uint32_t i = 0; i < members->member_count_; i++) {
    const auto& member = members->members_[i];
    if (std::string_view(member.name_) == field_name) return &member;
  }
  return nullptr;
}

bool IsRos2ScalarMember(
    const rosidl_typesupport_introspection_cpp::MessageMembers* members,
    std::string_view field_name,
    uint8_t type_id) {
  const auto* member = FindRos2Member(members, field_name);
  return member != nullptr && member->type_id_ == type_id && !member->is_array_;
}

const rosidl_typesupport_introspection_cpp::MessageMembers* GetRos2SubMemberMembers(
    const rosidl_typesupport_introspection_cpp::MessageMember* member) {
  if (member == nullptr ||
      member->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE ||
      member->is_array_ ||
      member->members_ == nullptr) {
    return nullptr;
  }
  return GetRosMembersInfo(member->members_);
}

bool IsRos2TimeSchema(const rosidl_typesupport_introspection_cpp::MessageMembers* members) {
  return members != nullptr &&
         IsRos2ScalarMember(members, "seconds", rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32) &&
         IsRos2ScalarMember(members, "fraction", rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32);
}

bool IsRos2MessageMemberWithSchema(
    const rosidl_typesupport_introspection_cpp::MessageMembers* members,
    std::string_view field_name,
    bool (*schema_checker)(const rosidl_typesupport_introspection_cpp::MessageMembers*)) {
  return schema_checker(GetRos2SubMemberMembers(FindRos2Member(members, field_name)));
}

bool IsRos2AgiHeaderSchema(const rosidl_typesupport_introspection_cpp::MessageMembers* members) {
  return members != nullptr && std::string_view(members->message_name_) == kAgiHeaderTypeName &&
         IsRos2ScalarMember(members, "seq_num", rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32) &&
         IsRos2ScalarMember(members, "publisher_name", rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING) &&
         IsRos2MessageMemberWithSchema(members, "publish_time", IsRos2TimeSchema);
}

bool IsRos2AgiRequestHeaderSchema(const rosidl_typesupport_introspection_cpp::MessageMembers* members) {
  return members != nullptr && std::string_view(members->message_name_) == kAgiRequestHeaderTypeName &&
         IsRos2ScalarMember(members, "request_id", rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32) &&
         IsRos2ScalarMember(members, "client_name", rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING) &&
         IsRos2MessageMemberWithSchema(members, "request_time", IsRos2TimeSchema);
}

Ros2FieldResult FindRos2SubMessageField(
    const void* custom_type_support_ptr,
    bool (*schema_checker)(const rosidl_typesupport_introspection_cpp::MessageMembers*)) {
  if (!custom_type_support_ptr) return {};

  const auto* ts =
      static_cast<const rosidl_message_type_support_t*>(custom_type_support_ptr);

  const auto* members = GetRosMembersInfo(ts);
  if (!members) return {};

  for (uint32_t i = 0; i < members->member_count_; i++) {
    const auto& member = members->members_[i];
    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE &&
        member.members_ != nullptr) {
      const auto* sub_members = GetRos2SubMemberMembers(&member);
      if (schema_checker(sub_members)) {
        return {true, member.offset_, sub_members};
      }
    }
  }
  return {};
}

size_t FindRos2MemberOffset(
    const rosidl_typesupport_introspection_cpp::MessageMembers* members,
    std::string_view field_name) {
  const auto* member = FindRos2Member(members, field_name);
  return member != nullptr ? member->offset_ : static_cast<size_t>(-1);
}

const rosidl_typesupport_introspection_cpp::MessageMembers* FindRos2SubMemberMembers(
    const rosidl_typesupport_introspection_cpp::MessageMembers* members,
    std::string_view field_name) {
  if (!members) return nullptr;
  for (uint32_t i = 0; i < members->member_count_; i++) {
    if (std::string_view(members->members_[i].name_) == field_name &&
        members->members_[i].type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE &&
        members->members_[i].members_ != nullptr) {
      return GetRos2SubMemberMembers(&(members->members_[i]));
    }
  }
  return nullptr;
}

void FillRos2TimeField(uint8_t* time_ptr,
                       const rosidl_typesupport_introspection_cpp::MessageMembers* time_members,
                       uint64_t timestamp_ns) {
  if (!time_members) return;
  AgiTime t = NanosToAgiTime(timestamp_ns);

  size_t seconds_offset = IsRos2ScalarMember(time_members, "seconds", rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32)
                              ? FindRos2MemberOffset(time_members, "seconds")
                              : static_cast<size_t>(-1);
  size_t fraction_offset = IsRos2ScalarMember(time_members, "fraction", rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32)
                               ? FindRos2MemberOffset(time_members, "fraction")
                               : static_cast<size_t>(-1);

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
    const auto* field = FindPbSubMessageField(custom_type_support_ptr, IsPbAgiHeaderSchema);
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
    auto result = FindRos2SubMessageField(custom_type_support_ptr, IsRos2AgiHeaderSchema);
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
    const auto* field = FindPbSubMessageField(custom_type_support_ptr, IsPbAgiRequestHeaderSchema);
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
    auto result = FindRos2SubMessageField(custom_type_support_ptr, IsRos2AgiRequestHeaderSchema);
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

    if (seq_field && seq_field->type() == google::protobuf::FieldDescriptor::TYPE_UINT32)
      header_refl->SetUInt32(header, seq_field, seq_num);
    if (name_field && name_field->type() == google::protobuf::FieldDescriptor::TYPE_STRING)
      header_refl->SetString(header, name_field, std::string(publisher_name));
    if (time_field && time_field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE)
      FillPbTimeField(header, time_field, publish_time_ns);
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

    size_t seq_offset = IsRos2ScalarMember(members, "seq_num", rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32)
                            ? FindRos2MemberOffset(members, "seq_num")
                            : static_cast<size_t>(-1);
    size_t name_offset = IsRos2ScalarMember(members, "publisher_name", rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING)
                             ? FindRos2MemberOffset(members, "publisher_name")
                             : static_cast<size_t>(-1);

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

    if (id_field && id_field->type() == google::protobuf::FieldDescriptor::TYPE_UINT32)
      header_refl->SetUInt32(header, id_field, request_id);
    if (name_field && name_field->type() == google::protobuf::FieldDescriptor::TYPE_STRING)
      header_refl->SetString(header, name_field, std::string(client_name));
    if (time_field && time_field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE)
      FillPbTimeField(header, time_field, request_time_ns);
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

    size_t id_offset = IsRos2ScalarMember(members, "request_id", rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32)
                           ? FindRos2MemberOffset(members, "request_id")
                           : static_cast<size_t>(-1);
    size_t name_offset = IsRos2ScalarMember(members, "client_name", rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING)
                             ? FindRos2MemberOffset(members, "client_name")
                             : static_cast<size_t>(-1);

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
