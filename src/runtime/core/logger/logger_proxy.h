// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <thread>

#include "aimrt_module_c_interface/logger/logger_base.h"
#include "core/logger/logger_backend_base.h"

#if __GLIBC__ == 2 && __GLIBC_MINOR__ < 30
  #include <sys/syscall.h>
  #define gettid() syscall(SYS_gettid)
#endif

namespace aimrt::runtime::core::logger {

class LoggerProxy {
 public:
  LoggerProxy(
      std::string_view module_name,
      aimrt_log_level_t lvl,
      const std::vector<std::unique_ptr<LoggerBackendBase>>& logger_backend_vec)
      : module_name_(module_name),
        lvl_(lvl),
        logger_backend_vec_(logger_backend_vec),
        base_(GenBase(this)) {}

  ~LoggerProxy() = default;

  LoggerProxy(const LoggerProxy&) = delete;
  LoggerProxy& operator=(const LoggerProxy&) = delete;

  const aimrt_logger_base_t* NativeHandle() const { return &base_; }

  // 这里不用atomic，也不用加锁，对修改的实时性要求不高
  aimrt_log_level_t LogLevel() const { return lvl_; }
  void SetLogLevel(aimrt_log_level_t lvl) { lvl_ = lvl; }

 private:
  aimrt_log_level_t GetLogLevel() const { return lvl_; }

  void Log(aimrt_log_level_t lvl,
           uint32_t line,
           uint32_t column,
           const char* file_name,
           const char* function_name,
           const char* log_data,
           size_t log_data_size) const {
    if (lvl >= lvl_) {
#if defined(_WIN32)
      thread_local size_t tid(std::hash<std::thread::id>{}(std::this_thread::get_id()));
#else
      thread_local size_t tid(gettid());
#endif

      LogDataWrapper log_data_wrapper{
          .module_name = module_name_,
          .thread_id = tid,
          .t = std::chrono::system_clock::now(),
          .lvl = lvl,
          .line = line,
          .column = column,
          .file_name = file_name,
          .function_name = function_name,
          .log_data = log_data,
          .log_data_size = log_data_size};

      for (const auto& logger_backend_ptr : logger_backend_vec_) {
        logger_backend_ptr->Log(log_data_wrapper);
      }
    }
  }

  static aimrt_logger_base_t GenBase(void* impl) {
    return aimrt_logger_base_t{
        .get_log_level = [](void* impl) -> aimrt_log_level_t {
          return static_cast<LoggerProxy*>(impl)->GetLogLevel();
        },
        .log = [](void* impl,
                  aimrt_log_level_t lvl,
                  uint32_t line,
                  uint32_t column,
                  const char* file_name,
                  const char* function_name,
                  const char* log_data,
                  size_t log_data_size) {
          static_cast<LoggerProxy*>(impl)->Log(
              lvl,
              line,
              column,
              file_name,
              function_name,
              log_data,
              log_data_size);  //
        },
        .impl = impl};
  }

 private:
  const std::string module_name_;
  aimrt_log_level_t lvl_;
  const std::vector<std::unique_ptr<LoggerBackendBase>>& logger_backend_vec_;

  const aimrt_logger_base_t base_;
};

}  // namespace aimrt::runtime::core::logger