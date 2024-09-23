// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_c_interface/util/buffer_base.h"
#include "aimrt_module_c_interface/util/string.h"

extern "C" {

/**
 * @brief Type support interface
 *
 */
typedef struct {
  /**
   * @brief Get type name
   * @note
   * Input 1: Implement pointer
   * Output: Type name
   */
  aimrt_string_view_t (*type_name)(void* impl);

  /**
   * @brief Function to create msg
   * @note
   * Input 1: Implement pointer
   * Output: Pointer to msg
   */
  void* (*create)(void* impl);

  /**
   * @brief Function to destroy msg
   * @note
   * Input 1: Implement pointer
   * Input 2: Pointer to msg
   */
  void (*destroy)(void* impl, void* msg);

  /**
   * @brief Function to copy msg
   * @note
   * Input 1: Implement pointer
   * Input 2: Pointer to the msg to be copied from
   * Input 3: Pointer to the msg to be copied to
   */
  void (*copy)(void* impl, const void* from, void* to);

  /**
   * @brief Function to move msg
   * @note
   * Input 1: Implement pointer
   * Input 2: Pointer to the msg to be moved from
   * Input 3: Pointer to the msg to be moved to
   */
  void (*move)(void* impl, void* from, void* to);

  /**
   * @brief Function to serialize the msg
   * @note
   * Input 1: Implement pointer
   * Input 2: Serialization type, eg: pb/json
   * Input 3: Pointer to the msg to be serialized
   * Input 4: Pointer to allocator for buffer array
   * Input 5: Pointer to the buffer array
   * Output: Serialization result
   */
  bool (*serialize)(
      void* impl,
      aimrt_string_view_t serialization_type,
      const void* msg,
      const aimrt_buffer_array_allocator_t* allocator,
      aimrt_buffer_array_t* buffer_array);

  /**
   * @brief Function to deserialize the msg
   * @note
   * Input 1: Implement pointer
   * Input 2: Serialization type, eg: pb/json
   * Input 3: Buffer array
   * Input 4: Pointer to the msg to be deserialized
   * Output: Deserialization result
   */
  bool (*deserialize)(
      void* impl,
      aimrt_string_view_t serialization_type,
      aimrt_buffer_array_view_t buffer_array_view,
      void* msg);

  /**
   * @brief Number of serialization types supported
   * @note
   * Input 1: Implement pointer
   * Output: Number of serialization types supported
   */
  size_t (*serialization_types_supported_num)(void* impl);

  /**
   * @brief List of serialization types supported
   * @note
   * Input 1: Implement pointer
   * Output: Name array. The length of this array is defined by serialization_types_supported_num
   */
  const aimrt_string_view_t* (*serialization_types_supported_list)(void* impl);

  /**
   * @brief For custom type support
   * @note
   * Input 1: Implement pointer
   * Output: Custom type support
   */
  const void* (*custom_type_support_ptr)(void* impl);

  /// Implement pointer
  void* impl;
} aimrt_type_support_base_t;
}
