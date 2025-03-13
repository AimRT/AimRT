// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Log level definition
typedef enum {
  AIMRT_LOG_LEVEL_TRACE = 0,
  AIMRT_LOG_LEVEL_DEBUG = 1,
  AIMRT_LOG_LEVEL_INFO = 2,
  AIMRT_LOG_LEVEL_WARN = 3,
  AIMRT_LOG_LEVEL_ERROR = 4,
  AIMRT_LOG_LEVEL_FATAL = 5,
  AIMRT_LOG_LEVEL_OFF = 6,
} aimrt_log_level_t;

/**
 * @brief Logger interface
 *
 */
typedef struct {
  /**
   * @brief Function to get log level
   *
   */
  aimrt_log_level_t (*get_log_level)(void* impl);

  /**
   * @brief Function to log
   * @note
   * Input 1: Implement pointer to logger handle
   * Input 2: Log level
   * Input 3: Code line
   * Input 4: Code file name
   * Input 5: Code function name
   * Input 6: Log data
   * Input 7: Log data size
   */
  void (*log)(void* impl, aimrt_log_level_t lvl, uint32_t line,
              const char* file_name, const char* function_name,
              const char* log_data, size_t log_data_size);

  /// Implement pointer
  void* impl;
} aimrt_logger_base_t;

#ifdef __cplusplus
}
#endif
