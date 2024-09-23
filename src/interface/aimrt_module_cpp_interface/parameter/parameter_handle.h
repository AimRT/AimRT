// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>

#include "aimrt_module_c_interface/parameter/parameter_handle_base.h"
#include "aimrt_module_cpp_interface/util/function.h"
#include "aimrt_module_cpp_interface/util/string.h"
#include "util/exception.h"

namespace aimrt::parameter {

using ParameterValReleaseCallback = util::Function<aimrt_function_parameter_val_release_callback_ops_t>;

class ParameterHandleRef {
 public:
  ParameterHandleRef() = default;
  explicit ParameterHandleRef(const aimrt_parameter_handle_base_t* base_ptr)
      : base_ptr_(base_ptr) {}
  ~ParameterHandleRef() = default;

  explicit operator bool() const { return (base_ptr_ != nullptr); }

  const aimrt_parameter_handle_base_t* NativeHandle() const {
    return base_ptr_;
  }

  std::string GetParameter(std::string_view key) const {
    AIMRT_ASSERT(base_ptr_, "Reference is null.");
    auto view_holder = base_ptr_->get_parameter(base_ptr_->impl, util::ToAimRTStringView(key));
    if (view_holder.parameter_val.len) {
      std::string result = util::ToStdString(view_holder.parameter_val);
      ParameterValReleaseCallback(view_holder.release_callback)();
      return result;
    }

    return "";
  }

  void SetParameter(std::string_view key, std::string_view val) {
    AIMRT_ASSERT(base_ptr_, "Reference is null.");
    base_ptr_->set_parameter(base_ptr_->impl, util::ToAimRTStringView(key), util::ToAimRTStringView(val));
  }

 private:
  const aimrt_parameter_handle_base_t* base_ptr_ = nullptr;
};

}  // namespace aimrt::parameter
