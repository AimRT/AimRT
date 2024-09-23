// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>

#include "aimrt_module_c_interface/configurator/configurator_base.h"
#include "aimrt_module_cpp_interface/util/string.h"

namespace aimrt::runtime::core::configurator {

class ConfiguratorProxy {
 public:
  explicit ConfiguratorProxy(std::string_view config_file_path = "")
      : config_file_path_(config_file_path),
        base_(GenBase(this)) {}
  ~ConfiguratorProxy() = default;

  ConfiguratorProxy(const ConfiguratorProxy&) = delete;
  ConfiguratorProxy& operator=(const ConfiguratorProxy&) = delete;

  const aimrt_configurator_base_t* NativeHandle() const { return &base_; }

 private:
  static aimrt_configurator_base_t GenBase(void* impl) {
    return aimrt_configurator_base_t{
        .config_file_path = [](void* impl) -> aimrt_string_view_t {
          return aimrt::util::ToAimRTStringView(static_cast<ConfiguratorProxy*>(impl)->config_file_path_);
        },
        .impl = impl};
  }

 private:
  const std::string config_file_path_;
  const aimrt_configurator_base_t base_;
};

}  // namespace aimrt::runtime::core::configurator
