// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_c_interface/allocator/allocator_base.h"
#include "aimrt_module_c_interface/channel/channel_handle_base.h"
#include "aimrt_module_c_interface/configurator/configurator_base.h"
#include "aimrt_module_c_interface/executor/executor_manager_base.h"
#include "aimrt_module_c_interface/logger/logger_base.h"
#include "aimrt_module_c_interface/parameter/parameter_handle_base.h"
#include "aimrt_module_c_interface/rpc/rpc_handle_base.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AIMRT module info
 * @note Version standard see https://semver.org/
 */
typedef struct {
  aimrt_string_view_t name;

  uint32_t major_version;

  uint32_t minor_version;

  uint32_t patch_version;

  uint32_t build_version;

  aimrt_string_view_t author;

  aimrt_string_view_t description;

} aimrt_module_info_t;

/**
 * @brief AIMRT runtime framework interface
 *
 */
typedef struct {
  /**
   * @brief Function to get module info for current core instance
   * @note
   * Parameter definition:
   * Input 1: Pointer to impl
   * Output: Module info
   */
  aimrt_module_info_t (*info)(void* impl);

  /**
   * @brief Function to get configurator handle
   * @note
   * Parameter definition:
   * Input 1: Pointer to impl
   * Output: Configurator handle
   */
  const aimrt_configurator_base_t* (*configurator)(void* impl);

  /**
   * @brief Function to get executor manager handle
   * @note
   * Parameter definition:
   * Input 1: Pointer to impl
   * Output: Executor manager handle
   */
  const aimrt_executor_manager_base_t* (*executor_manager)(void* impl);

  /**
   * @brief Function to get logger handle
   * @note
   * Parameter definition:
   * Input 1: Pointer to impl
   * Output: Logger handle
   */
  const aimrt_logger_base_t* (*logger)(void* impl);

  /**
   * @brief Function to get allocator handle
   * @note
   * Parameter definition:
   * Input 1: Pointer to impl
   * Output: Allocator handle
   */
  const aimrt_allocator_base_t* (*allocator_handle)(void* impl);

  /**
   * @brief Function to get rpc handle
   * @note
   * Parameter definition:
   * Input 1: Pointer to impl
   * Output: Rpc handle
   */
  const aimrt_rpc_handle_base_t* (*rpc_handle)(void* impl);

  /**
   * @brief Function to get channel handle
   * @note
   * Parameter definition:
   * Input 1: Pointer to impl
   * Output: Channel handle
   */
  const aimrt_channel_handle_base_t* (*channel_handle)(void* impl);

  /**
   * @brief Function to get parameter handle
   * @note
   * Parameter definition:
   * Input 1: Pointer to impl
   * Output: Parameter handle
   */
  const aimrt_parameter_handle_base_t* (*parameter_handle)(void* impl);

  /// Implement pointer
  void* impl;
} aimrt_core_base_t;

#ifdef __cplusplus
}
#endif
