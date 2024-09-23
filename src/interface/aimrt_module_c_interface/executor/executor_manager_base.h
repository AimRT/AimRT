// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_c_interface/executor/executor_base.h"
#include "aimrt_module_c_interface/util/string.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Executor manager interface
 *
 */
typedef struct {
  /**
   * @brief Function to get executor
   * @note
   * Input 1: Implement pointer to executor manager handle
   * Input 2: Executor name
   */
  const aimrt_executor_base_t* (*get_executor)(void* impl, aimrt_string_view_t executor_name);

  /// Implement pointer
  void* impl;
} aimrt_executor_manager_base_t;

#ifdef __cplusplus
}
#endif
