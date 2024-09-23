// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/parameter/parameter_handle.h"
#include "aimrt_module_cpp_interface/util/function.h"
#include "aimrt_module_cpp_interface/util/string.h"
#include "core/parameter/parameter_handle.h"

namespace aimrt::runtime::core::parameter {

class ParameterHandleProxy {
 public:
  explicit ParameterHandleProxy(ParameterHandle& parameter_handle)
      : base_(GenBase(&parameter_handle)) {}
  ~ParameterHandleProxy() = default;

  ParameterHandleProxy(const ParameterHandleProxy&) = delete;
  ParameterHandleProxy& operator=(const ParameterHandleProxy&) = delete;

  const aimrt_parameter_handle_base_t* NativeHandle() const { return &base_; }

 private:
  static aimrt_parameter_handle_base_t GenBase(void* impl) {
    return aimrt_parameter_handle_base_t{
        .get_parameter = [](void* impl, aimrt_string_view_t key) -> aimrt_parameter_val_view_holder_t {
          auto ptr = static_cast<ParameterHandle*>(impl)->GetParameter(aimrt::util::ToStdStringView(key));
          if (!ptr) [[unlikely]] {
            return aimrt_parameter_val_view_holder_t{
                .parameter_val = aimrt_string_view_t{nullptr, 0},
                .release_callback = nullptr};
          }

          auto* f = new aimrt::parameter::ParameterValReleaseCallback();
          (*f) = [ptr, f]() { delete f; };

          return aimrt_parameter_val_view_holder_t{
              .parameter_val = aimrt::util::ToAimRTStringView(*ptr),
              .release_callback = f->NativeHandle()};
        },
        .set_parameter = [](void* impl, aimrt_string_view_t key, aimrt_string_view_t val) {
          auto ptr = std::make_shared<std::string>(aimrt::util::ToStdString(val));
          static_cast<ParameterHandle*>(impl)->SetParameter(aimrt::util::ToStdStringView(key), ptr);  //
        },
        .impl = impl};
  }

 private:
  const aimrt_parameter_handle_base_t base_;
};

}  // namespace aimrt::runtime::core::parameter
