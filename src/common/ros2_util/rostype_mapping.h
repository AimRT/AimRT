// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>
#include <vector>

#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

// mapping ros type to cpp type
namespace aimrt::common::ros2_util {

template <int RosTypeId>
struct TypeMappingCpp {};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT> {
  using CppType = float;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE> {
  using CppType = double;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE> {
  using CppType = long double;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR> {
  using CppType = uint8_t;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR> {
  using CppType = uint16_t;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN> {
  using CppType = bool;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET> {
  using CppType = uint8_t;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8> {
  using CppType = uint8_t;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8> {
  using CppType = int8_t;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16> {
  using CppType = uint16_t;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16> {
  using CppType = int16_t;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32> {
  using CppType = uint32_t;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32> {
  using CppType = int32_t;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64> {
  using CppType = uint64_t;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64> {
  using CppType = int64_t;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING> {
  using CppType = std::string;
  using SequenceType = std::vector<CppType>;
};

template <>
struct TypeMappingCpp<rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING> {
  using CppType = std::u16string;
  using SequenceType = std::vector<CppType>;
};
}  // namespace aimrt::common::ros2_util