// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/util/type_support_pkg_loader.h"
#include <cstdio>
#include "util/exception.h"
#include "util/format.h"

namespace aimrt::runtime::core::util {

using DynlibGetTypeSupportArrayLengthFunc = size_t (*)();
using DynlibGetTypeSupportArray = const aimrt_type_support_base_t** (*)();

static constexpr const char* kDynlibGetTypeSupportArrayLengthFuncName = "AimRTDynlibGetTypeSupportArrayLength";
static constexpr const char* kDynlibGetTypeSupportArrayFuncName = "AimRTDynlibGetTypeSupportArray";

void TypeSupportPkgLoader::LoadTypeSupportPkg(std::string_view path) {
  path_ = path;
  if (!dynamic_lib_.Load(path_)) {
    throw aimrt::common::util::AimRTException(::aimrt_fmt::format(
        "Load dynamic lib failed, lib path {}, error info {}",
        path_, aimrt::common::util::DynamicLib::GetErr()));
  }
  auto* get_length_func = dynamic_lib_.GetSymbol(kDynlibGetTypeSupportArrayLengthFuncName);
  if (get_length_func == nullptr) {
    throw aimrt::common::util::AimRTException(::aimrt_fmt::format(
        "Cannot find symbol '{}' in lib {}.",
        kDynlibGetTypeSupportArrayLengthFuncName, path_));
  }

  auto* get_array_func = dynamic_lib_.GetSymbol(kDynlibGetTypeSupportArrayFuncName);
  if (get_array_func == nullptr) {
    throw aimrt::common::util::AimRTException(::aimrt_fmt::format(
        "Cannot find symbol '{}' in lib {}.",
        kDynlibGetTypeSupportArrayFuncName, path_));
  }

  size_t array_size = ((DynlibGetTypeSupportArrayLengthFunc)get_length_func)();
  const aimrt_type_support_base_t** array_ptr = ((DynlibGetTypeSupportArray)get_array_func)();

  if (array_ptr == nullptr) {
    throw aimrt::common::util::AimRTException(::aimrt_fmt::format(
        "Cannot get type support array in lib {}.", path_));
  }

  type_support_array_ = std::span{array_ptr, array_size};
}

void TypeSupportPkgLoader::UnLoadTypeSupportPkg() {
  if (!dynamic_lib_.IsLoaded()) return;

  dynamic_lib_.Unload();
}

}  // namespace aimrt::runtime::core::util
