// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  /**
   * @brief Get thread local buf from core
   *
   */
  void* (*get_thread_local_buf)(void* impl, size_t buf_size);

  /// Implement pointer
  void* impl;
} aimrt_allocator_base_t;

#ifdef __cplusplus
}
#endif
