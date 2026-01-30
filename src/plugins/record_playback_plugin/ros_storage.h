// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>
#include <string>

namespace aimrt::plugins::record_playback_plugin {

// The following source code is from ros2 humble.
// Copied and modified from
// https://github.com/ros2/rosidl/blob/humble/rosidl_runtime_c/include/rosidl_runtime_c/message_type_support_struct.h
// https://github.com/ros2/rosidl/blob/humble/rosidl_typesupport_introspection_c/include/rosidl_typesupport_introspection_c/message_introspection.h
// https://github.com/ros2/rosidl/blob/humble/rosidl_typesupport_introspection_c/include/rosidl_typesupport_introspection_c/field_types.h

typedef struct rosidl_message_type_support_t rosidl_message_type_support_t;

typedef const rosidl_message_type_support_t *(*rosidl_message_typesupport_handle_function)(
    const rosidl_message_type_support_t *, const char *);

/// Contains rosidl message type support data
struct rosidl_message_type_support_t {
  /// String identifier for the type_support.
  const char *typesupport_identifier;
  /// Pointer to the message type support library
  const void *data;
  /// Pointer to the message type support handler function
  rosidl_message_typesupport_handle_function func;
};

typedef struct MessageMember_s {
  /// The name of the field.
  const char *name_;
  /// The type of the field as a value of the field types enum,
  /// e.g.rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT
  uint8_t type_id_;
  /// If the field is a string, the upper bound on the length of the string.
  size_t string_upper_bound_;
  /// If the type_id_ value is rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE
  /// this points to an array describing the fields of the sub-interface.
  const rosidl_message_type_support_t *members_;
#if RCLCPP_VERSION_MAJOR == 28
  /// True if this field is annotated as `@key`, false otherwise.
  /// @note Only available in Jazzy and later versions.
  bool is_key_;
#endif
  /// True if this field is an array, false if it is a unary type. An array has the same value for
  /// type_id_.
  bool is_array_;
  /// If is_array_ is true, this contains the number of members in the array.
  size_t array_size_;
  /// If is_array_ is true, this specifies if the array has a maximum size. If it is true, the
  /// value in array_size_ is the maximum size.
  bool is_upper_bound_;
  /// The bytes into the interface's in-memory representation that this field can be found at.
  uint32_t offset_;
  /// If the interface has a default value, this points to it.
  const void *default_value_;
  /// If is_array_ is true, a pointer to a function that gives the size of array members.
  /// First argument should be a pointer to the actual memory representation of the member.
  size_t (*size_function)(const void *);
  /// If is_array_ is true, a pointer to a function that gives a const pointer to the member of the
  /// array indicated by index.
  /// First argument should be a pointer to the actual memory representation of the member.
  const void *(*get_const_function)(const void *, size_t index);
  /// If is_array_ is true, a pointer to a function that gives a pointer to the member of the
  /// array indicated by index.
  /// First argument should be a pointer to the actual memory representation of the member.
  void *(*get_function)(void *, size_t index);
  /// Pointer to a function that fetches (i.e. copies) an item from
  /// an array or sequence member. It takes a pointer to the member,
  /// an index (which is assumed to be valid), and a pointer to a
  /// pre-allocated value (which is assumed to be of the correct type).
  ///
  /// Available for array and sequence members.
  void (*fetch_function)(const void *, size_t index, void *);
  /// Pointer to a function that assigns (i.e. copies) a value to an
  /// item in an array or sequence member. It takes a pointer to the
  /// member, an index (which is assumed to be valid), and a pointer
  /// to an initialized value (which is assumed to be of the correct
  /// type).
  ///
  /// Available for array and sequence members.
  void (*assign_function)(void *, size_t index, const void *);
  /// If is_array_ is true, a pointer to a function that resizes the array.
  /// First argument should be a pointer to the actual memory representation of the member.
  void (*resize_function)(void *, size_t size);
} MessageMember;

enum class MessageInitialization {
  ALL = 0,
  SKIP = 1,
  ZERO = 2,
  DEFAULTS_ONLY = 3,
};

typedef struct MessageMembers_s {
  /// The namespace in which the interface resides, e.g. for
  /// example_messages/msg the namespaces generated would be
  /// example_message::msg".
  const char *message_namespace_;
  /// The name of the interface, e.g. "Int16"
  const char *message_name_;
  /// The number of fields in the interface
  uint32_t member_count_;
  /// The size of the interface structure in memory
  size_t size_of_;
#if RCLCPP_VERSION_MAJOR == 28
  /// True if any member of this message or any nested message is annotated as @key.
  /// @note Only available in Jazzy and later versions.
  bool has_any_key_member_;
#endif
  /// A pointer to the array that describes each field of the interface
  const MessageMember *members_;
  /// The function used to initialise the interface's in-memory representation
  void (*init_function)(void *, MessageInitialization);
  /// The function used to clean up the interface's in-memory representation
  void (*fini_function)(void *);
} MessageMembers;

enum rosidl_typesupport_introspection_c_field_types {
  /// Equivalent to float in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT = 1,
  /// Equivalent to double in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE = 2,
  /// Equivalent to long double in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE = 3,
  /// Equivalent to unsigned char in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_CHAR = 4,
  /// Equivalent to char16_t in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR = 5,
  /// Equivalent to _Bool in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN = 6,
  /// Equivalent to unsigned char in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_OCTET = 7,
  /// Equivalent to uint8_t in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_UINT8 = 8,
  /// Equivalent to int8_t in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_INT8 = 9,
  /// Equivalent to uint16_t in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_UINT16 = 10,
  /// Equivalent to int16_t in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_INT16 = 11,
  /// Equivalent to uint32_t in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_UINT32 = 12,
  /// Equivalent to int32_t in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_INT32 = 13,
  /// Equivalent to uint64_t in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_UINT64 = 14,
  /// Equivalent to int64_t in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_INT64 = 15,
  /// Equivalent to char * in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_STRING = 16,
  /// Equivalent to char16_t * in C types.
  rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING = 17,

  /// An embedded message type.
  rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE = 18,

  /// For backward compatibility only.
  rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT32 = 1,
  /// For backward compatibility only.
  rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT64 = 2,
  /// For backward compatibility only.
  rosidl_typesupport_introspection_c__ROS_TYPE_BOOL = 6,
  /// For backward compatibility only.
  rosidl_typesupport_introspection_c__ROS_TYPE_BYTE = 7
};

/// Equivalent to float in C++ types.
const uint8_t ROS_TYPE_FLOAT = rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT;
/// Equivalent to double in C++ types.
const uint8_t ROS_TYPE_DOUBLE = rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE;
/// Equivalent to long double in C++ types.
const uint8_t ROS_TYPE_LONG_DOUBLE = rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE;
/// Equivalent to unsigned char in C++ types.
const uint8_t ROS_TYPE_CHAR = rosidl_typesupport_introspection_c__ROS_TYPE_CHAR;
/// Equivalent to char16_t in C++ types.
const uint8_t ROS_TYPE_WCHAR = rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR;
/// Equivalent to bool in C++ types.
const uint8_t ROS_TYPE_BOOLEAN = rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN;
/// Equivalent to std::byte in C++ types.
const uint8_t ROS_TYPE_OCTET = rosidl_typesupport_introspection_c__ROS_TYPE_OCTET;
/// Equivalent to uint8_t in C++ types.
const uint8_t ROS_TYPE_UINT8 = rosidl_typesupport_introspection_c__ROS_TYPE_UINT8;
/// Equivalent to int8_t in C++ types.
const uint8_t ROS_TYPE_INT8 = rosidl_typesupport_introspection_c__ROS_TYPE_INT8;
/// Equivalent to uint16_t in C++ types.
const uint8_t ROS_TYPE_UINT16 = rosidl_typesupport_introspection_c__ROS_TYPE_UINT16;
/// Equivalent to int16_t in C++ types.
const uint8_t ROS_TYPE_INT16 = rosidl_typesupport_introspection_c__ROS_TYPE_INT16;
/// Equivalent to uint32_t in C++ types.
const uint8_t ROS_TYPE_UINT32 = rosidl_typesupport_introspection_c__ROS_TYPE_UINT32;
/// Equivalent to int32_t in C++ types.
const uint8_t ROS_TYPE_INT32 = rosidl_typesupport_introspection_c__ROS_TYPE_INT32;
/// Equivalent to uint64_t in C++ types.
const uint8_t ROS_TYPE_UINT64 = rosidl_typesupport_introspection_c__ROS_TYPE_UINT64;
/// Equivalent to int64_t in C++ types.
const uint8_t ROS_TYPE_INT64 = rosidl_typesupport_introspection_c__ROS_TYPE_INT64;
/// Equivalent to std::string in C++ types.
const uint8_t ROS_TYPE_STRING = rosidl_typesupport_introspection_c__ROS_TYPE_STRING;
/// Equivalent to std::u16string in C++ types.
const uint8_t ROS_TYPE_WSTRING = rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING;

/// An embedded message type.
const uint8_t ROS_TYPE_MESSAGE = rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE;

/// For backward compatibility only.
const uint8_t ROS_TYPE_BOOL = rosidl_typesupport_introspection_c__ROS_TYPE_BOOL;
/// For backward compatibility only.
const uint8_t ROS_TYPE_BYTE = rosidl_typesupport_introspection_c__ROS_TYPE_BYTE;
/// For backward compatibility only.
const uint8_t ROS_TYPE_FLOAT32 = rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT32;
/// For backward compatibility only.
const uint8_t ROS_TYPE_FLOAT64 = rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT64;

}  // namespace aimrt::plugins::record_playback_plugin