// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_c_interface/util/string.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configurator interface
 *
 */
typedef struct {
  /**
   * @brief Function to get config file path
   * @note
   * 1. If the module cfg file path is specified in the module cfg node, the
   * specified file path will be returned here.
   * 2. If the module's cfg is written in the root cfg file, the framework will
   * generate a temporary yaml configuration file containing module cfg info for
   * this module, and the path to this temporary configuration file will be
   * returned here.
   */
  aimrt_string_view_t (*config_file_path)(void* impl);

  /// Implement pointer
  void* impl;
} aimrt_configurator_base_t;

#ifdef __cplusplus
}
#endif
