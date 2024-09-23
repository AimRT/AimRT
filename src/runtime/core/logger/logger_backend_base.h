// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "core/logger/log_data_wrapper.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::runtime::core::logger {

class LoggerBackendBase {
 public:
  LoggerBackendBase() = default;
  virtual ~LoggerBackendBase() = default;

  LoggerBackendBase(const LoggerBackendBase&) = delete;
  LoggerBackendBase& operator=(const LoggerBackendBase&) = delete;

  virtual std::string_view Type() const noexcept = 0;  // It should always return the same value

  virtual void Initialize(YAML::Node options_node) = 0;
  virtual void Start() = 0;
  virtual void Shutdown() = 0;

  virtual std::list<std::pair<std::string, std::string>> GenInitializationReport() const noexcept { return {}; }

  virtual bool AllowDuplicates() const noexcept = 0;  // It should always return the same value

  /**
   * @brief Do the log
   * @note
   * 1. This method will only be called after 'Initialize' and before 'Shutdown'.
   * 2. Backend can define the actual behavior.
   *
   * @param log_data_wrapper Log data
   */
  virtual void Log(const LogDataWrapper& log_data_wrapper) noexcept = 0;
};

}  // namespace aimrt::runtime::core::logger
