// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <span>

#include "aimrt_module_c_interface/util/type_support_base.h"

#include "core/util/dynamic_lib.h"
#include "util/log_util.h"

namespace aimrt::plugins::record_playback_plugin {

class TypeSupportPkgLoader {
 public:
  TypeSupportPkgLoader() = default;
  ~TypeSupportPkgLoader() { UnLoadTypeSupportPkg(); }

  TypeSupportPkgLoader(const TypeSupportPkgLoader&) = delete;
  TypeSupportPkgLoader& operator=(const TypeSupportPkgLoader&) = delete;

  void LoadTypeSupportPkg(std::string_view path);

  void UnLoadTypeSupportPkg();

  auto GetTypeSupportArray() const { return type_support_array_; }

  auto& GetDynamicLib() { return dynamic_lib_; }

 private:
  std::string path_;
  aimrt::common::util::DynamicLib dynamic_lib_;

  std::span<const aimrt_type_support_base_t*> type_support_array_;
};

}  // namespace aimrt::plugins::record_playback_plugin
