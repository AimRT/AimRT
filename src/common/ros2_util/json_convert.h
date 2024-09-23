// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cassert>
#include <stdexcept>

#include <json/json.h>

#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "ros2_util/json_cpp_utils.h"
#include "ros2_util/rostype_mapping.h"
#include "ros2_util/type_support_util.h"

namespace aimrt::common::ros2_util {

namespace json_convert_impl {

inline bool IsSequence(const rosidl_typesupport_introspection_cpp::MessageMember &member) {
  return ((member.is_array_ && member.array_size_ == 0) || member.is_upper_bound_);
}

template <int RosTypeId>
inline void WriteSequenceMemberItem(const Json::Value &json, uint8_t *buffer) {
  using CppType = typename TypeMappingCpp<RosTypeId>::CppType;
  using SequenceType = typename TypeMappingCpp<RosTypeId>::SequenceType;
  reinterpret_cast<SequenceType *>(buffer)->push_back(GetJsonValue<CppType>(json));
}

template <>
inline void WriteSequenceMemberItem<rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL>(
    const Json::Value &json, uint8_t *buffer) {
  if (!json.isBool())
    throw std::runtime_error("WriteSequenceMemberItem<ROS_TYPE_BOOL>: json value is not bool");

  using SequenceType =
      typename TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL>::SequenceType;
  reinterpret_cast<SequenceType *>(buffer)->push_back(GetJsonValue<bool>(json));
}

template <>
inline void WriteSequenceMemberItem<rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET>(
    const Json::Value &json, uint8_t *buffer) {
  if (!json.isUInt())
    throw std::runtime_error("WriteSequenceMemberItem<ROS_TYPE_OCTET>(): json value is not uint");

  using SequenceType =
      typename TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET>::SequenceType;
  reinterpret_cast<SequenceType *>(buffer)->push_back(GetJsonValue<uint8_t>(json));
}

template <>
inline void WriteSequenceMemberItem<rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING>(
    const Json::Value &json, uint8_t *buffer) {
  if (!json.isString())
    throw std::runtime_error("WriteSequenceMemberItem<ROS_TYPE_WSTRING>(): json value is not string");

  throw std::runtime_error("Not support json to wstring type");
}

template <int RosTypeId>
inline void WriteSequence(
    const Json::Value &json,
    uint8_t *buffer,
    const rosidl_typesupport_introspection_cpp::MessageMember &member) {
  if (member.is_upper_bound_ && json.size() > member.array_size_)
    throw std::runtime_error("WriteSequence: upper bound exceeded");

  for (unsigned int i = 0; i < json.size(); i++) {
    WriteSequenceMemberItem<RosTypeId>(json[i], buffer);
  }
}

template <int RosTypeId>
inline void WriteMemberItem(const Json::Value &json, uint8_t *buffer) {
  using CppType = typename TypeMappingCpp<RosTypeId>::CppType;
  *reinterpret_cast<CppType *>(buffer) = GetJsonValue<CppType>(json);
}

template <>
inline void WriteMemberItem<rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING>(
    const Json::Value &json, uint8_t *buffer) {
  throw std::runtime_error("Not support json to wstring type");
}

// Convert a Json Node to Message
template <int RosTypeId>
inline void WriteMember(
    const Json::Value &json,
    uint8_t *buffer,
    const rosidl_typesupport_introspection_cpp::MessageMember &member) {
  using CppType = typename TypeMappingCpp<RosTypeId>::CppType;
  // Arrays and sequences have different struct representation.
  // An array is represented by a classic C array (pointer with data size == sizeof(type) * array_size).
  // Sequences on the other hand use a custom-defined struct with data, size and capacity members.

  // Handle sequences(std::vector)
  if (IsSequence(member)) {
    if (json[member.name_].isArray()) {
      WriteSequence<RosTypeId>(
          json[member.name_], buffer + member.offset_, member);
      return;
    }
    throw std::runtime_error("JsonToMessage: json member is not an array");
  }

  // Handle classic C arrays
  if (member.is_array_) {
    if (json[member.name_].isArray() && member.array_size_ == json[member.name_].size()) {
      for (unsigned int i = 0; i < member.array_size_; i++) {
        WriteMemberItem<RosTypeId>(
            json[member.name_][i],
            buffer + member.offset_ + sizeof(CppType) * i);
      }
      return;
    }
    throw std::runtime_error("JsonToMessage: json member is not an array or size not match");
  }

  // Handle single-item members
  WriteMemberItem<RosTypeId>(json[member.name_], buffer + member.offset_);
}

static void JsonToMessageImpl(
    const Json::Value &root,
    const rosidl_typesupport_introspection_cpp::MessageMembers *member_info,
    uint8_t *buffer);

inline void WriteMemberSequenceNested(
    const Json::Value &json,
    uint8_t *buffer,
    const rosidl_typesupport_introspection_cpp::MessageMember &member) {
  if (member.is_upper_bound_ && json.size() > member.array_size_)
    throw std::runtime_error("Json sequence is more than capacity");

  const auto *member_typeinfo =
      reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.members_->data);
  auto &seq = buffer;
  member.resize_function(seq, json.size());
  for (unsigned int i = 0; i < json.size(); i++) {
    JsonToMessageImpl(
        json[i],
        member_typeinfo,
        reinterpret_cast<uint8_t *>(member.get_function(seq, i)));
  }
}

inline void WriteMemberNested(
    const Json::Value &json,
    uint8_t *buffer,
    const rosidl_typesupport_introspection_cpp::MessageMember &member) {
  if (IsSequence(member)) {
    if (json[member.name_].isArray()) {
      WriteMemberSequenceNested(json[member.name_], buffer + member.offset_, member);
      return;
    }
    throw std::runtime_error("WriteMemberNested but the json is not array!");
  }

  const auto *member_typeinfo =
      reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.members_->data);
  if (member.is_array_) {
    for (unsigned int i = 0; i < json[member.name_].size(); i++) {
      JsonToMessageImpl(
          json[member.name_][i],
          member_typeinfo,
          buffer + member.offset_ + member_typeinfo->size_of_ * i);
    }
  } else {
    JsonToMessageImpl(json[member.name_], member_typeinfo, buffer + member.offset_);
  }
}

static void JsonToMessageImpl(
    const Json::Value &root,
    const rosidl_typesupport_introspection_cpp::MessageMembers *member_info,
    uint8_t *buffer) {
  for (uint32_t i = 0; i < member_info->member_count_; i++) {
    const auto &member = member_info->members_[i];

    if (!root.isMember(member.name_))
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

struct FakeVector {
  void *begin;
  void *end;
  void *end_capacity;
};

union vector_union {
  // The <Type> here doesn't matter because the memory layout (see fake_vector) is always the same.
  std::vector<int> *std;
  FakeVector *fake;
};

inline size_t GetVectorSize(const uint8_t *vector, size_t element_size) {
  // Should not get an element size of 0, but we want to avoid errors
  if (0ul == element_size)
    return 0ul;

  vector_union v = {reinterpret_cast<std::vector<int> *>(const_cast<uint8_t *>(vector))};
  return (reinterpret_cast<uint64_t>(v.fake->end) - reinterpret_cast<uint64_t>(v.fake->begin)) / element_size;
}

static void MessageToJsonImpl(
    const rosidl_typesupport_introspection_cpp::MessageMembers *member_info,
    const uint8_t *buffer,
    Json::Value &node);

inline void MemberToJsonArrayItem(
    const rosidl_typesupport_introspection_cpp::MessageMember &member_info,
    const uint8_t *member_data,
    Json::Value &array_node) {
  switch (member_info.type_id_) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      array_node.append(*reinterpret_cast<const float *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      array_node.append(*reinterpret_cast<const double *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
      array_node.append(*reinterpret_cast<const double *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      array_node.append(*reinterpret_cast<const uint8_t *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
      array_node.append(*reinterpret_cast<const uint16_t *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
      array_node.append(*reinterpret_cast<const bool *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
      array_node.append(*reinterpret_cast<const uint8_t *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      array_node.append(*reinterpret_cast<const uint8_t *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      array_node.append(*reinterpret_cast<const int8_t *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      array_node.append(*reinterpret_cast<const uint16_t *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      array_node.append(*reinterpret_cast<const int16_t *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      array_node.append(*reinterpret_cast<const uint32_t *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      array_node.append(*reinterpret_cast<const int32_t *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      array_node.append(*reinterpret_cast<const uint64_t *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      array_node.append(*reinterpret_cast<const int64_t *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
      array_node.append(*reinterpret_cast<const std::string *>(member_data));
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
      throw std::runtime_error("Not support wstring.");
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
      // For nested types, don't copy the data out of the buffer directly.
      // Recursively read the nested type into the YAML.
      Json::Value node;
      MessageToJsonImpl(
          reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member_info.members_->data),
          const_cast<uint8_t *>(member_data),
          node);
      array_node.append(node);
      break;
  }
}

template <typename T>
inline void DynamicArrayToJsonImpl(
    const rosidl_typesupport_introspection_cpp::MessageMember &member_info,
    const std::vector<T> *v,
    Json::Value &json_array) {
  std::vector<T> *vn = const_cast<std::vector<T> *>(v);
  for (size_t ii = 0; ii < vn->size(); ++ii) {
    MemberToJsonArrayItem(
        member_info,
        reinterpret_cast<const uint8_t *>(&(vn->data()[ii])),
        json_array);
  }
}

template <>
inline void DynamicArrayToJsonImpl(
    const rosidl_typesupport_introspection_cpp::MessageMember &member_info,
    const std::vector<bool> *v,
    Json::Value &json_array) {
  for (size_t ii = 0; ii < v->size(); ++ii) {
    json_array.append(v->operator[](ii));
  }
}

inline void DynamicArrayToJson(
    const rosidl_typesupport_introspection_cpp::MessageMember &member_info,
    const uint8_t *member_data,
    Json::Value &json_array) {
  switch (member_info.type_id_) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<float> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<double> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<long double> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<uint8_t> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<uint16_t> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<bool> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<uint8_t> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<uint8_t> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<int8_t> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<uint16_t> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<int16_t> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<uint32_t> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<int32_t> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<uint64_t> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<int64_t> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<std::string> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
      DynamicArrayToJsonImpl(member_info, reinterpret_cast<const std::vector<std::u16string> *>(member_data), json_array);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
      const auto *cur_member_info =
          reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member_info.members_->data);
      uint8_t *element_data = nullptr;
      memcpy(&element_data, member_data, sizeof(void *));
      size_t element_size = cur_member_info->size_of_;
      size_t element_count = GetVectorSize(member_data, element_size);
      for (size_t ii = 0; ii < element_count; ++ii) {
        Json::Value node;
        MessageToJsonImpl(cur_member_info, element_data + ii * element_size, node);
        json_array.append(node);
      }
      break;
  }
}

inline size_t SizeOfMemberType(uint8_t type_id) {
  switch (type_id) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      return sizeof(float);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      return sizeof(double);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
      return sizeof(long double);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      return sizeof(uint8_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
      return sizeof(uint16_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
      return sizeof(bool);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
      return sizeof(uint8_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      return sizeof(uint8_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      return sizeof(int8_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      return sizeof(uint16_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      return sizeof(int16_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      return sizeof(uint32_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      return sizeof(int32_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      return sizeof(uint64_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      return sizeof(int64_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
      return sizeof(std::string);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
      return sizeof(std::u16string);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
      throw std::runtime_error("Cannot get the size of a nested message.");
    default:
      throw std::runtime_error("Cannot get the size of an unknown message type.");
  }
}

inline void FixedArrayToJson(
    const rosidl_typesupport_introspection_cpp::MessageMember &member_info,
    const uint8_t *member_data,
    Json::Value &json_array) {
  size_t element_size{0};
  if (member_info.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
    element_size =
        reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member_info.members_->data)->size_of_;
  } else {
    element_size = SizeOfMemberType(member_info.type_id_);
  }
  for (size_t ii = 0; ii < member_info.array_size_; ++ii) {
    MemberToJsonArrayItem(member_info, &member_data[ii * element_size], json_array);
  }
}

// Convert an individual member's value from binary to json
inline void BasicValueToJson(
    const rosidl_typesupport_introspection_cpp::MessageMember &member_info,
    const uint8_t *member_data,
    Json::Value &member) {
  switch (member_info.type_id_) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      member = *reinterpret_cast<const float *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      member = *reinterpret_cast<const double *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
      member = *reinterpret_cast<const double *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      member = *reinterpret_cast<const uint8_t *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
      member = *reinterpret_cast<const uint16_t *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
      member = *reinterpret_cast<const bool *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
      member = *reinterpret_cast<const uint8_t *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      member = *reinterpret_cast<const uint8_t *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      member = *reinterpret_cast<const int8_t *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      member = *reinterpret_cast<const uint16_t *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      member = *reinterpret_cast<const int16_t *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      member = *reinterpret_cast<const uint32_t *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      member = *reinterpret_cast<const int32_t *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      member = *reinterpret_cast<const uint64_t *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      member = *reinterpret_cast<const int64_t *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
      member = *reinterpret_cast<const std::string *>(member_data);
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
      throw std::runtime_error("Not support wstring.");
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
      // For nested types, don't copy the data out of the buffer directly.
      // Recursively read the nested type into the YAML.
      MessageToJsonImpl(
          reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member_info.members_->data),
          const_cast<uint8_t *>(member_data),
          member);
      break;
    default:
      throw std::runtime_error("unknown type");
      break;
  }
}

inline void MemberToJson(
    const rosidl_typesupport_introspection_cpp::MessageMember &member_info,
    const uint8_t *member_data,
    Json::Value &node) {
  if (member_info.is_array_) {
    node = Json::Value(Json::arrayValue);
    if (member_info.is_upper_bound_ || member_info.array_size_ == 0) {
      // vector
      DynamicArrayToJson(member_info, member_data, node);
    } else {
      // c style list
      FixedArrayToJson(member_info, member_data, node);
    }
    return;
  }

  // ros msg
  if (member_info.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
    MessageToJsonImpl(
        reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member_info.members_->data),
        const_cast<uint8_t *>(member_data),
        node);
    return;
  }

  // basic msg
  BasicValueToJson(member_info, member_data, node);
}

static void MessageToJsonImpl(
    const rosidl_typesupport_introspection_cpp::MessageMembers *member_info,
    const uint8_t *buffer,
    Json::Value &node) {
  for (uint32_t ii = 0; ii < member_info->member_count_; ++ii) {
    const auto &cur_member_info = member_info->members_[ii];
    const uint8_t *member_data = &buffer[cur_member_info.offset_];
    MemberToJson(cur_member_info, member_data, node[cur_member_info.name_]);
  }
}

}  // namespace json_convert_impl

/**
 * @brief use typesupport convert json to ros msg
 *
 * @param json_str the json str
 * @param typesupport typesupport ptr
 * @param message the ros msg
 * @return true
 * @return false
 */
inline bool JsonToMessage(
    const std::string &json_str,
    const rosidl_message_type_support_t *typesupport,
    void *message) {
  using namespace json_convert_impl;

  if (message == nullptr) [[unlikely]]
    return false;

  const auto *member_info = GetRosMembersInfo(typesupport);
  if (member_info == nullptr) [[unlikely]]
    return false;

  Json::Reader reader;
  Json::Value root;

  if (!reader.parse(json_str, root)) [[unlikely]]
    return false;

  uint8_t *buffer = reinterpret_cast<uint8_t *>(message);

  try {
    JsonToMessageImpl(root, member_info, buffer);
  } catch (...) {
    return false;
  }

  return true;
}

/**
 * @brief use typesupport convert ros msg to json str
 *
 * @param message ros msg
 * @param typesupport typesuport ptr
 * @param json_str_res json result
 * @return true
 * @return false
 */
inline bool MessageToJson(
    const void *message,
    const rosidl_message_type_support_t *typesupport,
    std::string &json_str_res) {
  using namespace json_convert_impl;

  if (message == nullptr) [[unlikely]]
    return false;

  const auto *member_info = GetRosMembersInfo(typesupport);
  if (member_info == nullptr) [[unlikely]]
    return false;

  const uint8_t *buffer = reinterpret_cast<const uint8_t *>(message);

  Json::Value root;

  try {
    MessageToJsonImpl(member_info, buffer, root);
  } catch (...) {
    return false;
  }

  Json::FastWriter writer;
  json_str_res = writer.write(root);
  return true;
}

}  // namespace aimrt::common::ros2_util
