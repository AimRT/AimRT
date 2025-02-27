// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Buffer view
 *
 */
typedef struct {
  /// Const pointer to data array
  const void* data;

  /// Length of data
  size_t len;
} aimrt_buffer_view_t;

/**
 * @brief Buffer array view
 *
 */
typedef struct {
  /// Const pointer to buffer array
  const aimrt_buffer_view_t* data;

  /// Length of data
  size_t len;
} aimrt_buffer_array_view_t;

/**
 * @brief Buffer
 *
 */
typedef struct {
  /// Pointer to data array
  void* data;

  /// Length of data
  size_t len;
} aimrt_buffer_t;

/**
 * @brief Buffer array
 *
 */
typedef struct {
  /// Pointer to buffer array
  aimrt_buffer_t* data;

  /// Length of buffer array
  size_t len;

  /// Capacity of buffer array
  size_t capacity;

} aimrt_buffer_array_t;

/**
 * @brief Buffer array allocator interface
 * @note
 * A buffer array should use the same allocator to allocate and release memory
 */
typedef struct {
  /**
   * @brief Function to reserve new capacity for a buffer array
   * @note
   * 1. The new capacity size must be greater than or equal to the old value
   * 2. The value of buffer_array->data may changed after reserve
   * 3. Return false if reserve failed, and the value of buffer_array will not be changed
   *
   * Parameter definition:
   * Input 1: Pointer to impl
   * Input 2: Pointer to buffer array
   * Input 3: Size of new capacity
   * Output: True if reserve success, false if failed
   */
  bool (*reserve)(void* impl, aimrt_buffer_array_t* buffer_array, size_t new_cap);

  /**
   * @brief Function to allocate a new buffer for a buffer array
   * @note
   * 1. The length of buffer_array will increase by 1 after allocate
   * 2. The reserve function will be called if the capacity of buffer_array is too small
   * 3. The new buffer will be added to the end of buffer_array
   * 4. Return empty buffer if allocate failed
   *
   * Parameter definition:
   * Input 1: Pointer to impl
   * Input 2: Pointer to buffer array
   * Input 3: Size of new buffer
   * Output: New buffer
   */
  aimrt_buffer_t (*allocate)(void* impl, aimrt_buffer_array_t* buffer_array, size_t new_buffer_size);

  /**
   * @brief Function to release the buffer array
   * @note
   * 1. The value of buffer_array->data will be NULL after release
   * 2. The value of buffer_array->len and buffer_array->capacity will be zero after release
   *
   * Parameter definition:
   * Input 1: Pointer to impl
   * Input 2: Pointer to buffer array
   */
  void (*release)(void* impl, aimrt_buffer_array_t* buffer_array);

  /// Implement pointer
  void* impl;
} aimrt_buffer_array_allocator_t;

/**
 * @brief Buffer array with allocator
 *
 */
typedef struct {
  /// Pointer to buffer array
  aimrt_buffer_array_t* buffer_array;

  /// Pointer to allocator
  const aimrt_buffer_array_allocator_t* allocator;
} aimrt_buffer_array_with_allocator_t;

#ifdef __cplusplus
}
#endif
