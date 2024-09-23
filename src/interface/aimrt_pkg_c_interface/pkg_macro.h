// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string_view>

#include "aimrt_module_cpp_interface/module_base.h"
#include "aimrt_module_cpp_interface/util/string.h"
#include "aimrt_pkg_c_interface/pkg_main.h"

#define AIMRT_PKG_MAIN(__aimrt_module_register_array__)                                         \
  static constexpr size_t __aimrt_module_register_array_size__ =                                \
      sizeof(__aimrt_module_register_array__) / sizeof(__aimrt_module_register_array__[0]);     \
                                                                                                \
  extern "C" {                                                                                  \
                                                                                                \
  size_t AimRTDynlibGetModuleNum() {                                                            \
    return __aimrt_module_register_array_size__;                                                \
  }                                                                                             \
                                                                                                \
  const aimrt_string_view_t* AimRTDynlibGetModuleNameList() {                                   \
    static const aimrt_string_view_t* module_name_list = []() {                                 \
      static aimrt_string_view_t module_name_array[__aimrt_module_register_array_size__];       \
      for (size_t idx = 0; idx < __aimrt_module_register_array_size__; ++idx) {                 \
        module_name_array[idx] = aimrt::util::ToAimRTStringView(                                \
            std::get<0>(__aimrt_module_register_array__[idx]));                                 \
      }                                                                                         \
      return module_name_array;                                                                 \
    }();                                                                                        \
    return module_name_list;                                                                    \
  }                                                                                             \
                                                                                                \
  const aimrt_module_base_t* AimRTDynlibCreateModule(aimrt_string_view_t module_name) {         \
    std::string_view module_name_str = aimrt::util::ToStdStringView(module_name);               \
    for (size_t idx = 0; idx < __aimrt_module_register_array_size__; ++idx) {                   \
      std::string_view cur_module_name_str = std::get<0>(__aimrt_module_register_array__[idx]); \
      if (module_name_str == cur_module_name_str) {                                             \
        aimrt::ModuleBase* module_ptr = (std::get<1>(__aimrt_module_register_array__[idx]))();  \
        return module_ptr->NativeHandle();                                                      \
      }                                                                                         \
    }                                                                                           \
    return nullptr;                                                                             \
  }                                                                                             \
                                                                                                \
  void AimRTDynlibDestroyModule(const aimrt_module_base_t* module_ptr) {                        \
    delete static_cast<aimrt::ModuleBase*>(module_ptr->impl);                                   \
  }                                                                                             \
  }
