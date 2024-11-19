// Copyright(c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include "pybind11/pybind11.h"

#include <cstddef>

#include "rcutils/allocator.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/u16string.h"
#include "rosidl_runtime_c/u16string_functions.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"

namespace aimrt::runtime::python_runtime {

namespace py = pybind11;

// The following code is adapted from ros2 rclpy.
// see: https://github.com/ros2/rclpy/blob/humble/rclpy/src/rclpy/utils.cpp

typedef void destroy_ros_message_function(void*);
typedef void* create_ros_message_function();
typedef bool convert_from_py_function(PyObject*, void*);
typedef PyObject* convert_to_py_function(void*);

inline void* common_get_type_support(py::object pymessage) {
  py::object pymetaclass = pymessage.attr("__class__");

  py::object value = pymetaclass.attr("_TYPE_SUPPORT");
  auto* capsule_ptr = static_cast<void*>(value.cast<py::capsule>());

  return capsule_ptr;
}

inline std::unique_ptr<void, destroy_ros_message_function*>
create_from_py(py::object pymessage) {
  py::object pymetaclass = pymessage.attr("__class__");

  py::object value = pymetaclass.attr("_CREATE_ROS_MESSAGE");
  auto capsule_ptr = static_cast<void*>(value.cast<py::capsule>());
  auto create_ros_message =
      reinterpret_cast<create_ros_message_function*>(capsule_ptr);
  if (!create_ros_message) {
    throw py::error_already_set();
  }

  value = pymetaclass.attr("_DESTROY_ROS_MESSAGE");
  capsule_ptr = static_cast<void*>(value.cast<py::capsule>());
  auto destroy_ros_message =
      reinterpret_cast<destroy_ros_message_function*>(capsule_ptr);
  if (!destroy_ros_message) {
    throw py::error_already_set();
  }

  void* message = create_ros_message();
  if (!message) {
    throw std::bad_alloc();
  }
  return std::unique_ptr<
      void, destroy_ros_message_function*>(message, destroy_ros_message);
}

inline std::unique_ptr<void, destroy_ros_message_function*>
convert_from_py(py::object pymessage) {
  std::unique_ptr<void, destroy_ros_message_function*> message =
      create_from_py(pymessage);

  py::object pymetaclass = pymessage.attr("__class__");

  auto capsule_ptr = static_cast<void*>(
      pymetaclass.attr("_CONVERT_FROM_PY").cast<py::capsule>());
  auto convert =
      reinterpret_cast<convert_from_py_function*>(capsule_ptr);
  if (!convert) {
    throw py::error_already_set();
  }

  if (!convert(pymessage.ptr(), message.get())) {
    throw py::error_already_set();
  }

  return message;
}

inline create_ros_message_function* get_create_ros_message_function(py::object pyclass) {
  py::object pymetaclass = pyclass.attr("__class__");
  auto* capsule_ptr = static_cast<void*>(pymetaclass.attr("_CREATE_ROS_MESSAGE").cast<py::capsule>());
  auto create = reinterpret_cast<create_ros_message_function*>(capsule_ptr);
  if (!create) {
    throw py::error_already_set();
  }
  return create;
}

inline destroy_ros_message_function* get_destroy_ros_message_function(py::object pyclass) {
  py::object pymetaclass = pyclass.attr("__class__");
  auto* capsule_ptr = static_cast<void*>(pymetaclass.attr("_DESTROY_ROS_MESSAGE").cast<py::capsule>());
  auto destroy = reinterpret_cast<destroy_ros_message_function*>(capsule_ptr);
  if (!destroy) {
    throw py::error_already_set();
  }
  return destroy;
}

inline convert_to_py_function* get_convert_to_py_function(py::object pyclass) {
  py::object pymetaclass = pyclass.attr("__class__");
  auto* capsule_ptr = static_cast<void*>(pymetaclass.attr("_CONVERT_TO_PY").cast<py::capsule>());
  auto convert = reinterpret_cast<convert_to_py_function*>(capsule_ptr);
  if (!convert) {
    throw py::error_already_set();
  }
  return convert;
}

inline convert_from_py_function* get_convert_from_py_function(py::object pyclass) {
  py::object pymetaclass = pyclass.attr("__class__");
  auto* capsule_ptr = static_cast<void*>(pymetaclass.attr("_CONVERT_FROM_PY").cast<py::capsule>());
  auto convert = reinterpret_cast<convert_from_py_function*>(capsule_ptr);
  if (!convert) {
    throw py::error_already_set();
  }
  return convert;
}

// End of adapted code from ros2 rclpy.

inline const rosidl_message_type_support_t* ImproveToIntrospectionTypeSupport(
    const rosidl_message_type_support_t* type_support) {
  return get_message_typesupport_handle(type_support, rosidl_typesupport_introspection_c__identifier);
}

inline const rosidl_typesupport_introspection_c__MessageMembers* GetRosMembersInfo(
    const rosidl_message_type_support_t* type_support) {
  // TODO(zhangyi1357): Remove the temporary ts variable.
  const auto* ts = ImproveToIntrospectionTypeSupport(type_support);
  if (!ts) {
    throw std::runtime_error("Failed to get introspection type support.");
  }
  return reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(ts->data);
}

template <enum rosidl_typesupport_introspection_c_field_types T>
struct RosTypeMapping;

#define ROS_TYPE_ID(ros_type) \
  rosidl_typesupport_introspection_c__ROS_TYPE_##ros_type

#define ROS_SEQUENCE_TYPE(name_in_sequence) \
  rosidl_runtime_c__##name_in_sequence##__Sequence

#define ROS_SEQUENCE_COPY_FUNCTION(name_in_sequence) \
  rosidl_runtime_c__##name_in_sequence##__Sequence__copy

// There is no move function for sequence in ROS2.
// So we implement these functions ourself in the above.
#define ROS_SEQUENCE_MOVE_FUNCTION(name_in_sequence) \
  rosidl_runtime_c__##name_in_sequence##__Sequence__move

#define DEFINE_ROS_BASIC_TYPE_MAPPING(ros_type, name_in_sequence, c_type)                      \
  template <>                                                                                  \
  struct RosTypeMapping<ROS_TYPE_ID(ros_type)> {                                               \
    using CType = c_type;                                                                      \
    using SequenceType = ROS_SEQUENCE_TYPE(name_in_sequence);                                  \
    static constexpr auto SequenceCopyFunction = ROS_SEQUENCE_COPY_FUNCTION(name_in_sequence); \
    static void SequenceMoveFunction(SequenceType* from_seq, SequenceType* to_seq) {           \
      auto allocator = rcutils_get_default_allocator();                                        \
      allocator.deallocate(to_seq->data, allocator.state);                                     \
      *to_seq = *from_seq;                                                                     \
      memset(from_seq, 0, sizeof(SequenceType));                                               \
    }                                                                                          \
  };

#define DEFINE_ROS_STRING_TYPE_MAPPING(ros_type, name_in_sequence, c_type)                     \
  template <>                                                                                  \
  struct RosTypeMapping<ROS_TYPE_ID(ros_type)> {                                               \
    using CType = c_type;                                                                      \
    using SequenceType = ROS_SEQUENCE_TYPE(name_in_sequence);                                  \
    static constexpr auto SequenceCopyFunction = ROS_SEQUENCE_COPY_FUNCTION(name_in_sequence); \
    static void SequenceMoveFunction(SequenceType* from_seq, SequenceType* to_seq) {           \
      auto allocator = rcutils_get_default_allocator();                                        \
      for (size_t ii = 0; ii < to_seq->capacity; ++ii) {                                       \
        allocator.deallocate(to_seq->data[ii].data, allocator.state);                          \
      }                                                                                        \
      allocator.deallocate(to_seq->data, allocator.state);                                     \
      *to_seq = *from_seq;                                                                     \
      memset(from_seq, 0, sizeof(SequenceType));                                               \
    }                                                                                          \
  };

DEFINE_ROS_BASIC_TYPE_MAPPING(FLOAT, float, float)
DEFINE_ROS_BASIC_TYPE_MAPPING(DOUBLE, double, double)
DEFINE_ROS_BASIC_TYPE_MAPPING(LONG_DOUBLE, long_double, long double)
DEFINE_ROS_BASIC_TYPE_MAPPING(CHAR, char, unsigned char)
DEFINE_ROS_BASIC_TYPE_MAPPING(WCHAR, wchar, char16_t)
DEFINE_ROS_BASIC_TYPE_MAPPING(BOOLEAN, boolean, bool)
DEFINE_ROS_BASIC_TYPE_MAPPING(OCTET, octet, std::byte)
DEFINE_ROS_BASIC_TYPE_MAPPING(UINT8, uint8, uint8_t)
DEFINE_ROS_BASIC_TYPE_MAPPING(INT8, int8, int8_t)
DEFINE_ROS_BASIC_TYPE_MAPPING(UINT16, uint16, uint16_t)
DEFINE_ROS_BASIC_TYPE_MAPPING(INT16, int16, int16_t)
DEFINE_ROS_BASIC_TYPE_MAPPING(UINT32, uint32, uint32_t)
DEFINE_ROS_BASIC_TYPE_MAPPING(INT32, int32, int32_t)
DEFINE_ROS_BASIC_TYPE_MAPPING(UINT64, uint64, uint64_t)
DEFINE_ROS_BASIC_TYPE_MAPPING(INT64, int64, int64_t)
DEFINE_ROS_STRING_TYPE_MAPPING(STRING, String, rosidl_runtime_c__String)
DEFINE_ROS_STRING_TYPE_MAPPING(WSTRING, U16String, rosidl_runtime_c__U16String)

void CopyRosMessage(const rosidl_typesupport_introspection_c__MessageMembers* members, const void* from, void* to);

#define COPY_ROS_BASIC_TYPE(ros_type)                                                     \
  case ROS_TYPE_ID(ros_type):                                                             \
    *reinterpret_cast<RosTypeMapping<ROS_TYPE_ID(ros_type)>::CType*>(to_ptr) =            \
        *reinterpret_cast<const RosTypeMapping<ROS_TYPE_ID(ros_type)>::CType*>(from_ptr); \
    break;

inline void CopyBasicMember(
    const rosidl_typesupport_introspection_c__MessageMember& member,
    const void* from_ptr, void* to_ptr) {
  switch (member.type_id_) {
    COPY_ROS_BASIC_TYPE(FLOAT)
    COPY_ROS_BASIC_TYPE(DOUBLE)
    COPY_ROS_BASIC_TYPE(LONG_DOUBLE)
    COPY_ROS_BASIC_TYPE(CHAR)
    COPY_ROS_BASIC_TYPE(WCHAR)
    COPY_ROS_BASIC_TYPE(BOOLEAN)
    COPY_ROS_BASIC_TYPE(OCTET)
    COPY_ROS_BASIC_TYPE(UINT8)
    COPY_ROS_BASIC_TYPE(INT8)
    COPY_ROS_BASIC_TYPE(UINT16)
    COPY_ROS_BASIC_TYPE(INT16)
    COPY_ROS_BASIC_TYPE(UINT32)
    COPY_ROS_BASIC_TYPE(INT32)
    COPY_ROS_BASIC_TYPE(UINT64)
    COPY_ROS_BASIC_TYPE(INT64)
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING: {
      rosidl_runtime_c__String__copy(reinterpret_cast<const rosidl_runtime_c__String*>(from_ptr),
                                     reinterpret_cast<rosidl_runtime_c__String*>(to_ptr));
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING: {
      rosidl_runtime_c__U16String__copy(reinterpret_cast<const rosidl_runtime_c__U16String*>(from_ptr),
                                        reinterpret_cast<rosidl_runtime_c__U16String*>(to_ptr));
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE: {
      const auto* sub_members =
          reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(member.members_->data);
      CopyRosMessage(sub_members, from_ptr, to_ptr);
      break;
    }
    default:
      throw std::runtime_error("Unknown basic type: " + std::string(member.name_) +
                               " with type id: " + std::to_string(member.type_id_));
  }
}

#define COPY_ROS_BASIC_TYPE_ARRAY(ros_type)                                                                   \
  case ROS_TYPE_ID(ros_type):                                                                                 \
    std::memcpy(to_ptr, from_ptr, member.array_size_ * sizeof(RosTypeMapping<ROS_TYPE_ID(ros_type)>::CType)); \
    break;

inline void CopyStaticSizeArray(
    const rosidl_typesupport_introspection_c__MessageMember& member,
    const void* from_ptr, void* to_ptr) {
  switch (member.type_id_) {
    COPY_ROS_BASIC_TYPE_ARRAY(FLOAT)
    COPY_ROS_BASIC_TYPE_ARRAY(DOUBLE)
    COPY_ROS_BASIC_TYPE_ARRAY(LONG_DOUBLE)
    COPY_ROS_BASIC_TYPE_ARRAY(CHAR)
    COPY_ROS_BASIC_TYPE_ARRAY(WCHAR)
    COPY_ROS_BASIC_TYPE_ARRAY(BOOLEAN)
    COPY_ROS_BASIC_TYPE_ARRAY(OCTET)
    COPY_ROS_BASIC_TYPE_ARRAY(UINT8)
    COPY_ROS_BASIC_TYPE_ARRAY(INT8)
    COPY_ROS_BASIC_TYPE_ARRAY(UINT16)
    COPY_ROS_BASIC_TYPE_ARRAY(INT16)
    COPY_ROS_BASIC_TYPE_ARRAY(UINT32)
    COPY_ROS_BASIC_TYPE_ARRAY(INT32)
    COPY_ROS_BASIC_TYPE_ARRAY(UINT64)
    COPY_ROS_BASIC_TYPE_ARRAY(INT64)
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING: {
      auto* to_str = reinterpret_cast<rosidl_runtime_c__String*>(to_ptr);
      const auto* from_str = reinterpret_cast<const rosidl_runtime_c__String*>(from_ptr);
      for (size_t i = 0; i < member.array_size_; ++i) {
        rosidl_runtime_c__String__copy(&from_str[i], &to_str[i]);
      }
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING: {
      auto* to_wstr = reinterpret_cast<rosidl_runtime_c__U16String*>(to_ptr);
      const auto* from_wstr = reinterpret_cast<const rosidl_runtime_c__U16String*>(from_ptr);
      for (size_t i = 0; i < member.array_size_; ++i) {
        rosidl_runtime_c__U16String__copy(&from_wstr[i], &to_wstr[i]);
      }
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE: {
      const auto* sub_members =
          reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(member.members_->data);
      if (!member.get_function) [[unlikely]] {
        throw std::runtime_error("Failed to get get function for message: " + std::string(member.name_));
      }
      for (size_t ii = 0; ii < member.array_size_; ++ii) {
        CopyRosMessage(sub_members,
                       member.get_function(const_cast<void*>(from_ptr), ii),
                       member.get_function(to_ptr, ii));
      }
      break;
    }
    default:
      throw std::runtime_error("Unknown array type: " + std::string(member.name_) +
                               " with type id: " + std::to_string(member.type_id_));
  }
}

#define COPY_ROS_SEQUENCE(ros_type)                                                                                \
  case ROS_TYPE_ID(ros_type): {                                                                                    \
    auto* to_seq = reinterpret_cast<RosTypeMapping<ROS_TYPE_ID(ros_type)>::SequenceType*>(to_ptr);                 \
    const auto* from_seq = reinterpret_cast<const RosTypeMapping<ROS_TYPE_ID(ros_type)>::SequenceType*>(from_ptr); \
    RosTypeMapping<ROS_TYPE_ID(ros_type)>::SequenceCopyFunction(from_seq, to_seq);                                 \
    break;                                                                                                         \
  }

inline void CopyDynamicSizeArray(
    const rosidl_typesupport_introspection_c__MessageMember& member,
    const void* from_ptr, void* to_ptr) {
  switch (member.type_id_) {
    COPY_ROS_SEQUENCE(FLOAT)
    COPY_ROS_SEQUENCE(DOUBLE)
    COPY_ROS_SEQUENCE(LONG_DOUBLE)
    COPY_ROS_SEQUENCE(CHAR)
    COPY_ROS_SEQUENCE(WCHAR)
    COPY_ROS_SEQUENCE(BOOLEAN)
    COPY_ROS_SEQUENCE(OCTET)
    COPY_ROS_SEQUENCE(UINT8)
    COPY_ROS_SEQUENCE(INT8)
    COPY_ROS_SEQUENCE(UINT16)
    COPY_ROS_SEQUENCE(INT16)
    COPY_ROS_SEQUENCE(UINT32)
    COPY_ROS_SEQUENCE(INT32)
    COPY_ROS_SEQUENCE(UINT64)
    COPY_ROS_SEQUENCE(INT64)
    COPY_ROS_SEQUENCE(STRING)
    COPY_ROS_SEQUENCE(WSTRING)
    case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE: {
      const auto* sub_members =
          reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(member.members_->data);
      if (!member.get_function) [[unlikely]] {
        throw std::runtime_error("Failed to get get function for message: " + std::string(member.name_));
      }
      if (!member.resize_function) [[unlikely]] {
        throw std::runtime_error("Failed to get resize function for message: " + std::string(member.name_));
      }
      if (!member.size_function) [[unlikely]] {
        throw std::runtime_error("Failed to get size function for message: " + std::string(member.name_));
      }
      auto from_size = member.size_function(from_ptr);
      member.resize_function(to_ptr, from_size);
      for (size_t ii = 0; ii < from_size; ++ii) {
        CopyRosMessage(sub_members,
                       member.get_function(const_cast<void*>(from_ptr), ii),
                       member.get_function(to_ptr, ii));
      }
      break;
    }
    default:
      throw std::runtime_error("Unknown array type: " + std::string(member.name_) +
                               " with type id: " + std::to_string(member.type_id_));
  }
}

inline void CopyRosMessage(const rosidl_typesupport_introspection_c__MessageMembers* members,
                           const void* from, void* to) {
  for (size_t ii = 0; ii < members->member_count_; ++ii) {
    const auto& member = members->members_[ii];
    const void* from_ptr = static_cast<const void*>(static_cast<const uint8_t*>(from) + member.offset_);
    void* to_ptr = static_cast<void*>(static_cast<uint8_t*>(to) + member.offset_);
    if (!member.is_array_) {
      CopyBasicMember(member, from_ptr, to_ptr);
    } else if (member.array_size_ > 0 && !member.is_upper_bound_) {
      CopyStaticSizeArray(member, from_ptr, to_ptr);
    } else {
      CopyDynamicSizeArray(member, from_ptr, to_ptr);
    }
  }
}

template <typename StringType>
inline void MoveRosString(StringType* from_str, StringType* to_str) {
  auto allocator = rcutils_get_default_allocator();
  allocator.deallocate(to_str->data, allocator.state);
  *to_str = *from_str;
  memset(from_str, 0, sizeof(StringType));
}

void MoveRosMessage(const rosidl_typesupport_introspection_c__MessageMembers* members,
                    void* from, void* to);

inline void MoveBasicMember(const rosidl_typesupport_introspection_c__MessageMember& member,
                            void* from_ptr, void* to_ptr) {
  switch (member.type_id_) {
    COPY_ROS_BASIC_TYPE(FLOAT)
    COPY_ROS_BASIC_TYPE(DOUBLE)
    COPY_ROS_BASIC_TYPE(LONG_DOUBLE)
    COPY_ROS_BASIC_TYPE(CHAR)
    COPY_ROS_BASIC_TYPE(WCHAR)
    COPY_ROS_BASIC_TYPE(BOOLEAN)
    COPY_ROS_BASIC_TYPE(OCTET)
    COPY_ROS_BASIC_TYPE(UINT8)
    COPY_ROS_BASIC_TYPE(INT8)
    COPY_ROS_BASIC_TYPE(UINT16)
    COPY_ROS_BASIC_TYPE(INT16)
    COPY_ROS_BASIC_TYPE(UINT32)
    COPY_ROS_BASIC_TYPE(INT32)
    COPY_ROS_BASIC_TYPE(UINT64)
    COPY_ROS_BASIC_TYPE(INT64)
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING: {
      MoveRosString(reinterpret_cast<rosidl_runtime_c__String*>(from_ptr),
                    reinterpret_cast<rosidl_runtime_c__String*>(to_ptr));
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING: {
      MoveRosString(reinterpret_cast<rosidl_runtime_c__U16String*>(from_ptr),
                    reinterpret_cast<rosidl_runtime_c__U16String*>(to_ptr));
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE: {
      const auto* sub_members =
          reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(member.members_->data);
      MoveRosMessage(sub_members, from_ptr, to_ptr);
      break;
    }
    default:
      throw std::runtime_error("Unknown basic type: " + std::string(member.name_) +
                               " with type id: " + std::to_string(member.type_id_));
  }
}

inline void MoveStaticSizeArray(const rosidl_typesupport_introspection_c__MessageMember& member,
                                void* from_ptr, void* to_ptr) {
  switch (member.type_id_) {
    COPY_ROS_BASIC_TYPE_ARRAY(FLOAT)
    COPY_ROS_BASIC_TYPE_ARRAY(DOUBLE)
    COPY_ROS_BASIC_TYPE_ARRAY(LONG_DOUBLE)
    COPY_ROS_BASIC_TYPE_ARRAY(CHAR)
    COPY_ROS_BASIC_TYPE_ARRAY(WCHAR)
    COPY_ROS_BASIC_TYPE_ARRAY(BOOLEAN)
    COPY_ROS_BASIC_TYPE_ARRAY(OCTET)
    COPY_ROS_BASIC_TYPE_ARRAY(UINT8)
    COPY_ROS_BASIC_TYPE_ARRAY(INT8)
    COPY_ROS_BASIC_TYPE_ARRAY(UINT16)
    COPY_ROS_BASIC_TYPE_ARRAY(INT16)
    COPY_ROS_BASIC_TYPE_ARRAY(UINT32)
    COPY_ROS_BASIC_TYPE_ARRAY(INT32)
    COPY_ROS_BASIC_TYPE_ARRAY(UINT64)
    COPY_ROS_BASIC_TYPE_ARRAY(INT64)
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING: {
      auto* to_str = reinterpret_cast<rosidl_runtime_c__String*>(to_ptr);
      auto* from_str = reinterpret_cast<rosidl_runtime_c__String*>(from_ptr);
      for (size_t i = 0; i < member.array_size_; ++i) {
        MoveRosString(&from_str[i], &to_str[i]);
      }
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING: {
      auto* to_wstr = reinterpret_cast<rosidl_runtime_c__U16String*>(to_ptr);
      auto* from_wstr = reinterpret_cast<rosidl_runtime_c__U16String*>(from_ptr);
      for (size_t i = 0; i < member.array_size_; ++i) {
        MoveRosString(&from_wstr[i], &to_wstr[i]);
      }
      break;
    }
    case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE: {
      const auto* sub_members =
          reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(member.members_->data);
      if (!member.get_function) [[unlikely]] {
        throw std::runtime_error("Failed to get get function for message: " + std::string(member.name_));
      }
      for (size_t ii = 0; ii < member.array_size_; ++ii) {
        MoveRosMessage(sub_members,
                       member.get_function(from_ptr, ii),
                       member.get_function(to_ptr, ii));
      }
      break;
    }
    default:
      throw std::runtime_error("Unknown array type: " + std::string(member.name_) +
                               " with type id: " + std::to_string(member.type_id_));
  }
}

#define MOVE_ROS_SEQUENCE(ros_type)                                                                    \
  case ROS_TYPE_ID(ros_type): {                                                                        \
    auto* to_seq = reinterpret_cast<RosTypeMapping<ROS_TYPE_ID(ros_type)>::SequenceType*>(to_ptr);     \
    auto* from_seq = reinterpret_cast<RosTypeMapping<ROS_TYPE_ID(ros_type)>::SequenceType*>(from_ptr); \
    RosTypeMapping<ROS_TYPE_ID(ros_type)>::SequenceMoveFunction(from_seq, to_seq);                     \
    break;                                                                                             \
  }

inline void MoveDynamicSizeArray(const rosidl_typesupport_introspection_c__MessageMember& member,
                                 void* from_ptr, void* to_ptr) {
  switch (member.type_id_) {
    MOVE_ROS_SEQUENCE(FLOAT)
    MOVE_ROS_SEQUENCE(DOUBLE)
    MOVE_ROS_SEQUENCE(LONG_DOUBLE)
    MOVE_ROS_SEQUENCE(CHAR)
    MOVE_ROS_SEQUENCE(WCHAR)
    MOVE_ROS_SEQUENCE(BOOLEAN)
    MOVE_ROS_SEQUENCE(OCTET)
    MOVE_ROS_SEQUENCE(UINT8)
    MOVE_ROS_SEQUENCE(INT8)
    MOVE_ROS_SEQUENCE(UINT16)
    MOVE_ROS_SEQUENCE(INT16)
    MOVE_ROS_SEQUENCE(UINT32)
    MOVE_ROS_SEQUENCE(INT32)
    MOVE_ROS_SEQUENCE(UINT64)
    MOVE_ROS_SEQUENCE(INT64)
    MOVE_ROS_SEQUENCE(STRING)
    MOVE_ROS_SEQUENCE(WSTRING)
    case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE: {
      const auto* sub_members =
          reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(member.members_->data);
      if (!member.get_function) [[unlikely]] {
        throw std::runtime_error("Failed to get get function for message: " + std::string(member.name_));
      }
      if (!member.resize_function) [[unlikely]] {
        throw std::runtime_error("Failed to get resize function for message: " + std::string(member.name_));
      }
      if (!member.size_function) [[unlikely]] {
        throw std::runtime_error("Failed to get size function for message: " + std::string(member.name_));
      }
      auto from_size = member.size_function(from_ptr);
      member.resize_function(to_ptr, from_size);
      for (size_t ii = 0; ii < from_size; ++ii) {
        MoveRosMessage(sub_members,
                       member.get_function(from_ptr, ii),
                       member.get_function(to_ptr, ii));
      }
      break;
    }
    default:
      throw std::runtime_error("Unknown array type: " + std::string(member.name_) +
                               " with type id: " + std::to_string(member.type_id_));
  }
}

inline void MoveRosMessage(const rosidl_typesupport_introspection_c__MessageMembers* members,
                           void* from, void* to) {
  for (size_t ii = 0; ii < members->member_count_; ++ii) {
    const auto& member = members->members_[ii];
    void* from_ptr = static_cast<void*>(static_cast<uint8_t*>(from) + member.offset_);
    void* to_ptr = static_cast<void*>(static_cast<uint8_t*>(to) + member.offset_);
    if (!member.is_array_) {
      MoveBasicMember(member, from_ptr, to_ptr);
    } else if (member.array_size_ > 0 && !member.is_upper_bound_) {
      MoveStaticSizeArray(member, from_ptr, to_ptr);
    } else {
      MoveDynamicSizeArray(member, from_ptr, to_ptr);
    }
  }
}

#undef COPY_ROS_BASIC_TYPE
#undef COPY_ROS_STRING_TYPE
#undef COPY_ROS_BASIC_TYPE_ARRAY
#undef COPY_ROS_SEQUENCE
#undef MOVE_ROS_SEQUENCE

}  // namespace aimrt::runtime::python_runtime
