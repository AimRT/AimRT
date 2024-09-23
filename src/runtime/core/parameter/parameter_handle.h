// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "aimrt_module_c_interface/parameter/parameter_handle_base.h"
#include "util/log_util.h"
#include "util/string_util.h"

namespace aimrt::runtime::core::parameter {

class ParameterHandle {
 public:
  ParameterHandle()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~ParameterHandle() = default;

  ParameterHandle(const ParameterHandle&) = delete;
  ParameterHandle& operator=(const ParameterHandle&) = delete;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  std::shared_ptr<const std::string> GetParameter(std::string_view key);
  void SetParameter(std::string_view key, const std::shared_ptr<std::string>& value_ptr);
  void SetParameter(std::string_view key, const std::string& value) {
    SetParameter(key, std::make_shared<std::string>(value));
  }
  std::vector<std::string> ListParameter() const;

 private:
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  mutable std::mutex parameter_map_mutex_;
  std::unordered_map<
      std::string,
      std::shared_ptr<std::string>,
      aimrt::common::util::StringHash,
      std::equal_to<>>
      parameter_map_;
};

}  // namespace aimrt::runtime::core::parameter
