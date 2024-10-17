// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_c_interface/executor/executor_manager_base.h"
#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/util/string.h"

namespace aimrt::executor {

class ExecutorManagerRef {
 public:
  ExecutorManagerRef() = default;
  explicit ExecutorManagerRef(const aimrt_executor_manager_base_t* base_ptr)
      : base_ptr_(base_ptr) {}
  ~ExecutorManagerRef() = default;

  explicit operator bool() const { return (base_ptr_ != nullptr); }

  const aimrt_executor_manager_base_t* NativeHandle() const {
    return base_ptr_;
  }

  ExecutorRef GetExecutor(std::string_view executor_name) const {
    AIMRT_ASSERT(base_ptr_, "Reference is null.");
    return ExecutorRef(base_ptr_->get_executor(
        base_ptr_->impl, aimrt::util::ToAimRTStringView(executor_name)));
  }

 private:
  const aimrt_executor_manager_base_t* base_ptr_ = nullptr;
};

}  // namespace aimrt::executor