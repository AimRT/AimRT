// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <stdint.h>

#include "aimrt_module_c_interface/util/function_base.h"
#include "aimrt_module_c_interface/util/string.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Operate struct for executor task
 * @note
 * Signature: void(*)()
 */
typedef struct {
  void (*invoker)(void* object);
  void (*relocator)(void* from, void* to);
  void (*destroyer)(void* object);
} aimrt_function_executor_task_ops_t;

/**
 * @brief Executor interface
 *
 */
typedef struct {
  /// Function to get executor type
  aimrt_string_view_t (*type)(void* impl);

  /// Function to get executor name
  aimrt_string_view_t (*name)(void* impl);

  /// Function to get executor thread safety
  bool (*is_thread_safe)(void* impl);

  /// Function to determine if in current executor
  bool (*is_in_current_executor)(void* impl);

  /// Function to determine if in current executor
  bool (*is_support_timer_schedule)(void* impl);

  /**
   * @brief Function to execute task
   * @note
   * Input 1: Implement pointer to executor handle
   * Input 2: Task, which ops type is 'aimrt_function_executor_task_ops_t'
   */
  void (*execute)(void* impl, aimrt_function_base_t* task);

  /**
   * @brief Function to get current timestamp
   * @note
   * Input 1: Implement pointer to executor handle
   * Output: Current nanosecond timestamp, since 1970-01-01 00:00:00 UTC
   */
  uint64_t (*now)(void* impl);

  /**
   * @brief Function to execute task at a time point
   * @note
   * Input 1: Implement pointer to executor handle
   * Input 2: Timestamp, nanosecond since 1970-01-01 00:00:00 UTC
   * Input 2: Task, which ops type is 'aimrt_function_executor_task_ops_t'
   */
  void (*execute_at_ns)(void* impl, uint64_t tp, aimrt_function_base_t* task);

  /// Implement pointer
  void* impl;
} aimrt_executor_base_t;

#ifdef __cplusplus
}
#endif
