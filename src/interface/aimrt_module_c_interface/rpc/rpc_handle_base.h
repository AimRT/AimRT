// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_c_interface/rpc/rpc_context_base.h"
#include "aimrt_module_c_interface/util/function_base.h"
#include "aimrt_module_c_interface/util/string.h"
#include "aimrt_module_c_interface/util/type_support_base.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Operate struct for rpc callback
 * @note
 * Signature: void(*)(uint32_t status)
 * Input 1: RPC result status
 */
typedef struct {
  void (*invoker)(void* object, uint32_t status);
  void (*relocator)(void* from, void* to);
  void (*destroyer)(void* object);
} aimrt_function_rpc_callback_ops_t;

/// Operate struct for rpc service callback
typedef aimrt_function_rpc_callback_ops_t aimrt_function_service_callback_ops_t;

/// Operate struct for rpc client callback
typedef aimrt_function_rpc_callback_ops_t aimrt_function_client_callback_ops_t;

/**
 * @brief Operate struct for rpc service
 * @note
 * Signature:
 * void (*)(
 *     const aimrt_rpc_context_base_t* ctx_ptr,
 *     const void* req,
 *     void* rsp,
 *     aimrt_function_base_t* callback)
 * Input 1: Rpc server context
 * Input 2: Const pointer to req
 * Input 3: Pointer to rsp
 * Input 4: Rpc result callback, which ops type is 'aimrt_function_service_callback_ops_t'
 */
typedef struct {
  void (*invoker)(
      void* object,
      const aimrt_rpc_context_base_t* ctx_ptr,
      const void* req,
      void* rsp,
      aimrt_function_base_t* callback);
  void (*relocator)(void* from, void* to);
  void (*destroyer)(void* object);
} aimrt_function_service_func_ops_t;

/**
 * @brief RPC handle interface
 *
 */
typedef struct {
  /**
   * @brief Function to register rpc service function
   * @note
   * Input 1: Implement pointer to rpc handle
   * Input 2: Func name
   * Input 3: Custom type support
   * Input 4: Req type support
   * Input 5: Rsp type support
   * Input 6: Service_func
   * Output: Registration result
   */
  bool (*register_service_func)(
      void* impl,
      aimrt_string_view_t func_name,
      const void* custom_type_support_ptr,
      const aimrt_type_support_base_t* req_type_support,
      const aimrt_type_support_base_t* rsp_type_support,
      aimrt_function_base_t* service_func);

  /**
   * @brief Function to register rpc client function
   * @note
   * Input 1: Implement pointer to rpc handle
   * Input 2: Func name
   * Input 3: Custom type support
   * Input 4: Req type support
   * Input 5: Rsp type support
   * Output: Registration result
   */
  bool (*register_client_func)(
      void* impl,
      aimrt_string_view_t func_name,
      const void* custom_type_support_ptr,
      const aimrt_type_support_base_t* req_type_support,
      const aimrt_type_support_base_t* rsp_type_support);

  /**
   * @brief Function to invoke rpc
   * @note
   * Input 1: Implement pointer to rpc handle
   * Input 2: func name
   * Input 3: Rpc client context
   * Input 4: Const pointer to req
   * Input 5: Pointer to rsp
   * Input 5: Rpc Invoke callback, which ops type is 'aimrt_function_client_callback_ops_t'
   */
  void (*invoke)(
      void* impl,
      aimrt_string_view_t func_name,
      const aimrt_rpc_context_base_t* ctx_ptr,
      const void* req_ptr,
      void* rsp_ptr,
      aimrt_function_base_t* callback);

  /**
   * @brief Function to merge server context to client context
   * @note
   * Input 1: Implement pointer to rpc handle
   * Input 2: Rpc server context
   * Input 3: Rpc client context, for output
   */
  void (*merge_server_context_to_client_context)(
      void* impl,
      const aimrt_rpc_context_base_t* server_ctx_ptr,
      const aimrt_rpc_context_base_t* client_ctx_ptr);

  /// Implement pointer
  void* impl;
} aimrt_rpc_handle_base_t;

#ifdef __cplusplus
}
#endif
