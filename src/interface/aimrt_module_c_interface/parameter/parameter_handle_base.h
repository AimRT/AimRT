// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_c_interface/util/function_base.h"
#include "aimrt_module_c_interface/util/string.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Operate struct for parameter value release callback
 * @note Signature: void(*)()
 *
 */
typedef struct {
  void (*invoker)(void* object);
  void (*relocator)(void* from, void* to);
  void (*destroyer)(void* object);
} aimrt_function_parameter_val_release_callback_ops_t;

/// Parameter value with release callback
typedef struct {
  /// parameter value
  aimrt_string_view_t parameter_val;

  /**
   * @brief release callback
   * @note ops type is aimrt_function_parameter_val_release_callback_ops_t
   */
  aimrt_function_base_t* release_callback;
} aimrt_parameter_val_view_holder_t;

/// Parameter handle
typedef struct {
  /**
   * @brief Get parameter reference
   * @note
   * Input 1: Implement pointer to parameter handle
   * Input 2: Parameter key
   * Output: Parameter value with release callback
   */
  aimrt_parameter_val_view_holder_t (*get_parameter)(void* impl, aimrt_string_view_t key);

  /**
   * @brief Set parameter
   * @note
   * Input 1: Implement pointer to parameter handle
   * Input 2: Parameter key
   * Input 3: Parameter value
   */
  void (*set_parameter)(void* impl, aimrt_string_view_t key, aimrt_string_view_t val);

  /// Implement pointer
  void* impl;
} aimrt_parameter_handle_base_t;

#ifdef __cplusplus
}
#endif
