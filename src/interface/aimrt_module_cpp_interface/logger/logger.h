// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_c_interface/logger/logger_base.h"
#include "util/log_util.h"

namespace aimrt::logger {

class LoggerRef {
 public:
  LoggerRef() = default;
  explicit LoggerRef(const aimrt_logger_base_t* base_ptr)
      : base_ptr_(base_ptr) {}
  ~LoggerRef() = default;

  explicit operator bool() const { return (base_ptr_ != nullptr); }

  const aimrt_logger_base_t* NativeHandle() const { return base_ptr_; }

  uint32_t GetLogLevel() const {
    AIMRT_ASSERT(base_ptr_, "Reference is null.");
    return static_cast<uint32_t>(base_ptr_->get_log_level(base_ptr_->impl));
  }

  void Log(
      uint32_t lvl,
      uint32_t line,
      const char* file_name,
      const char* function_name,
      const char* log_data,
      size_t log_data_size) const {
    AIMRT_ASSERT(base_ptr_, "Reference is null.");
    base_ptr_->log(
        base_ptr_->impl,
        static_cast<aimrt_log_level_t>(lvl),
        line, file_name, function_name,
        log_data, log_data_size);
  };

 private:
  const aimrt_logger_base_t* base_ptr_ = nullptr;
};

inline LoggerRef GetSimpleLoggerRef() {
  static constexpr aimrt_logger_base_t kSimpleLogger{
      .get_log_level = [](void* impl) -> aimrt_log_level_t {
        return static_cast<aimrt_log_level_t>(aimrt::common::util::SimpleLogger::GetLogLevel());
      },
      .log = [](void* impl, aimrt_log_level_t lvl, uint32_t line,
                const char* file_name, const char* function_name,
                const char* log_data, size_t log_data_size) {
        aimrt::common::util::SimpleLogger::Log(
            static_cast<uint32_t>(lvl), line, file_name, function_name, log_data, log_data_size);  //
      }};

  return LoggerRef(&kSimpleLogger);
}

}  // namespace aimrt::logger
