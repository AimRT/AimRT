// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifndef AIMRT_FUNCTION_LOCAL_BUF_SIZE
  #define AIMRT_FUNCTION_LOCAL_BUF_SIZE (3 * sizeof(void*))
#endif

/**
 * @brief Function
 *
 */
typedef struct {
  /// Buffer to store closure data
  unsigned char object_buf[AIMRT_FUNCTION_LOCAL_BUF_SIZE];

  /**
   * @brief Const pointer to an operate struct
   * @note
   * An operate struct has 3 necessary members:
   * 1. invoker: Function to invoke the closure, the signature of it is the signature the closure
   * 2. relocator: Function to relocator the closure, signature: void(*relocator)(void* from, void* to);
   * 3. destroyer: Function to destroyer the closure, signature: void(*destroyer)(void* object);
   *
   * eg:
   * typedef struct {
   *   int (*invoker)(void* object, int val);
   *   void (*relocator)(void* from, void* to);
   *   void (*destroyer)(void* object);
   * } test_ops_t;
   *
   * This is a simple example of an operate struct that represents a simple closure.
   * It's invoke signature is <int(*)(int)>.
   */
  const void* ops;
} aimrt_function_base_t;

#ifdef __cplusplus
}
#endif
