// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_c_interface/core_base.h"
#include "aimrt_module_c_interface/util/string.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AIMRT module interface
 * Users need to implement these interfaces
 */
typedef struct {
  /**
   * @brief Function to get module info
   * @note
   * 1. Modules should return the same info throughout their entire lifecycle.
   *
   * Parameter definition:
   * Input 1: Pointer to module impl
   * Output: Module info
   */
  aimrt_module_info_t (*info)(void* impl);

  /**
   * @brief Function to initialize module
   * @note
   * 1. The framework will call the initialize method of module after loading it.
   * 2. The initialization order of modules is undefined.
   * 3. The framework ensures that the initialize method is executed in the main thread, and the module should not block this method for too long.
   * 4. Some api (eg: RPC Registration/Channel Registration) can only be called in the initialize method.
   *
   * Parameter definition:
   * Input 1: Pointer to module impl
   * Input 2: AimRT core interface pointer, modules call the function of the framework through this handle Output: Initialize result
   */
  bool (*initialize)(void* impl, const aimrt_core_base_t* core);

  /**
   * @brief Function to start module
   * @note
   * 1. The framework will call the start method of module after initializing all modules.
   * 2. The start sequence of the modules will be consistent with the initialization sequence of the modules.
   * 3. The framework ensures that the start method is executed in the main thread, and the module should not block this method for too long.
   * 4. Some api (eg: RPC invoke/Channel publish) can only be called after the start method.
   *
   * Parameter definition:
   * Input 1: Pointer to module impl
   * Output: Start result
   */
  bool (*start)(void* impl);

  /**
   * @brief Function to shutdown module
   * @note
   * 1. The framework will call the shutdown method of module when the framework stops running.
   * 2. The shutdown order of modules will be the opposite to the initialization order of modules.
   * 3. The framework ensures that the shutdown method is executed in the main thread, and the module should not block this method for too long.
   * 4. All api of framework should not be called after the shutdown method.
   *
   * Parameter definition:
   * Input 1: Pointer to module impl
   */
  void (*shutdown)(void* impl);

  /// Implement pointer
  void* impl;
} aimrt_module_base_t;

#ifdef __cplusplus
}
#endif
