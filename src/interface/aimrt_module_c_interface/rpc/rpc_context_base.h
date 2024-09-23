// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "stdint.h"

#include "aimrt_module_c_interface/util/string.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Some frame fields. Users should not directly modify these fields

#define AIMRT_RPC_CONTEXT_KEY_PREFIX "aimrt-"

/// eg: backend://uri_defined_by_backend
#define AIMRT_RPC_CONTEXT_KEY_TO_ADDR "aimrt-to_addr"

/// eg: json/pb
#define AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE "aimrt-serialization_type"

/// eg: pb:/example.ExampleService/GetFooData
#define AIMRT_RPC_CONTEXT_KEY_FUNCTION_NAME "aimrt-function_name"

/// eg: local/mqtt/http
#define AIMRT_RPC_CONTEXT_KEY_BACKEND "aimrt-backend"

typedef enum {
  AIMRT_RPC_CLIENT_CONTEXT = 0,
  AIMRT_RPC_SERVER_CONTEXT = 1,
} aimrt_rpc_context_type_t;

/**
 * @brief Rpc context operate interface
 *
 */
typedef struct {
  /**
   * @brief Function to check if the context is used already
   *
   */
  bool (*check_used)(void* impl);

  /**
   * @brief Function to set the context to used status
   *
   */
  void (*set_used)(void* impl);

  /**
   * @brief Function to get context type
   *
   */
  aimrt_rpc_context_type_t (*get_type)(void* impl);

  /**
   * @brief Function to get the timeout(ns)
   *
   */
  uint64_t (*get_timeout_ns)(void* impl);

  /**
   * @brief Function to set the timeout(ns)
   *
   */
  void (*set_timeout_ns)(void* impl, uint64_t timeout);

  /**
   * @brief Function to get kv meta data
   *
   */
  aimrt_string_view_t (*get_meta_val)(void* impl, aimrt_string_view_t key);

  /**
   * @brief Function to set kv meta data
   *
   */
  void (*set_meta_val)(void* impl, aimrt_string_view_t key, aimrt_string_view_t val);

  /**
   * @brief Function to get all meta keys
   *
   */
  aimrt_string_view_array_t (*get_meta_keys)(void* impl);

} aimrt_rpc_context_base_ops_t;

/**
 * @brief Rpc context interface
 *
 */
typedef struct {
  /// Const pointor to rpc context operate interface
  const aimrt_rpc_context_base_ops_t* ops;

  /// Implement pointer
  void* impl;
} aimrt_rpc_context_base_t;

#ifdef __cplusplus
}
#endif
