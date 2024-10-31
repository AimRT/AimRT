// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cassert>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "ros2_util/rostype_mapping.h"
#include "ros2_util/type_support_util.h"
#include "ros2_util/yaml_cpp_utils.h"

namespace aimrt::common::ros2_util {

namespace yaml_convert_impl {

inline bool IsSequence(const rosidl_typesupport_introspection_cpp::MessageMember &member) {
  return ((member.is_array_ && member.array_size_ == 0) || member.is_upper_bound_);
}

template <int RosTypeId>
inline void WriteSequenceMemberItem(const YAML::Node &yaml, uint8_t *buffer) {
  using CppType = typename TypeMappingCpp<RosTypeId>::CppType;
  using SequenceType = typename TypeMappingCpp<RosTypeId>::SequenceType;
  reinterpret_cast<SequenceType *>(buffer)->push_back(GetYamlValue<CppType>(yaml));
}

template <int RosTypeId>
inline void WriteSequence(
    const YAML::Node &yaml,
    uint8_t *buffer,
    const rosidl_typesupport_introspection_cpp::MessageMember &member) {
  if (member.is_upper_bound_ && yaml.size() > member.array_size_)
    throw std::runtime_error("WriteSequence: upper bound exceeded");

  for (unsigned int i = 0; i < yaml.size(); i++) {
    WriteSequenceMemberItem<RosTypeId>(yaml[i], buffer);
  }
}

template <int RosTypeId>
inline void WriteMemberItem(const YAML::Node &yaml, uint8_t *buffer) {
  using CppType = typename TypeMappingCpp<RosTypeId>::CppType;
  *reinterpret_cast<CppType *>(buffer) = GetYamlValue<CppType>(yaml);
}

template <int RosTypeId>
inline void WriteMember(
    const YAML::Node &yaml,
    uint8_t *buffer,
    const rosidl_typesupport_introspection_cpp::MessageMember &member) {
  using CppType = typename TypeMappingCpp<RosTypeId>::CppType;

  if (IsSequence(member)) {
    if (yaml[member.name_].IsSequence()) {
      WriteSequence<RosTypeId>(yaml[member.name_], buffer + member.offset_, member);
      return;
    }
    throw std::runtime_error("YamlToMessage: yaml member is not a sequence");
  }

  if (member.is_array_) {
    if (yaml[member.name_].IsSequence() && member.array_size_ == yaml[member.name_].size()) {
      for (unsigned int i = 0; i < member.array_size_; i++) {
        WriteMemberItem<RosTypeId>(yaml[member.name_][i], buffer + member.offset_ + sizeof(CppType) * i);
      }
      return;
    }
    throw std::runtime_error("YamlToMessage: yaml member is not a sequence or size not match");
  }

  WriteMemberItem<RosTypeId>(yaml[member.name_], buffer + member.offset_);
}

static void YamlToMessageImpl(
    const YAML::Node &root,
    const rosidl_typesupport_introspection_cpp::MessageMembers *member_info,
    uint8_t *buffer);

inline void WriteMemberSequenceNested(
    const YAML::Node &yaml,
    uint8_t *buffer,
    const rosidl_typesupport_introspection_cpp::MessageMember &member) {
  if (member.is_upper_bound_ && yaml.size() > member.array_size_)
    throw std::runtime_error("Yaml sequence is more than capacity");

  const auto *member_typeinfo =
      reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.members_->data);
  auto &seq = buffer;
  member.resize_function(seq, yaml.size());
  for (unsigned int i = 0; i < yaml.size(); i++) {
    YamlToMessageImpl(yaml[i], member_typeinfo,
                      reinterpret_cast<uint8_t *>(member.get_function(seq, i)));
  }
}

inline void WriteMemberNested(
    const YAML::Node &yaml,
    uint8_t *buffer,
    const rosidl_typesupport_introspection_cpp::MessageMember &member) {
  if (IsSequence(member)) {
    if (yaml[member.name_].IsSequence()) {
      WriteMemberSequenceNested(yaml[member.name_], buffer + member.offset_, member);
      return;
    }
    throw std::runtime_error("WriteMemberNested but the yaml is not sequence!");
  }

  const auto *member_typeinfo =
      reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.members_->data);
  if (member.is_array_) {
    for (unsigned int i = 0; i < yaml[member.name_].size(); i++) {
      YamlToMessageImpl(yaml[member.name_][i], member_typeinfo,
                        buffer + member.offset_ + member_typeinfo->size_of_ * i);
    }
  } else {
    YamlToMessageImpl(yaml[member.name_], member_typeinfo, buffer + member.offset_);
  }
}

static void YamlToMessageImpl(
    const YAML::Node &root,
    const rosidl_typesupport_introspection_cpp::MessageMembers *member_info,
    uint8_t *buffer) {
  for (uint32_t i = 0; i < member_info->member_count_; i++) {
    const auto &member = member_info->members_[i];

    if (!root[member.name_])
      continue;

    switch (member.type_id_) {
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        WriteMember<rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING>(root, buffer, member);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
        throw std::runtime_error("Not support wstring.");
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
        WriteMemberNested(root, buffer, member);
        break;
      default:
        throw std::runtime_error("Current ros msg type is not support.");
    }
  }
}

}  // namespace yaml_convert_impl

inline bool YamlToMessage(
    const std::string &yaml_str,
    const rosidl_message_type_support_t *typesupport,
    void *message) {
  using namespace yaml_convert_impl;

  if (message == nullptr) [[unlikely]]
    return false;

  const auto *member_info = GetRosMembersInfo(typesupport);
  if (member_info == nullptr) [[unlikely]]
    return false;

  YAML::Node root;
  try {
    root = YAML::Load(yaml_str);
  } catch (...) {
    return false;
  }

  uint8_t *buffer = reinterpret_cast<uint8_t *>(message);

  try {
    YamlToMessageImpl(root, member_info, buffer);
  } catch (...) {
    return false;
  }

  return true;
}

}  // namespace aimrt::common::ros2_util