// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string_view>

#include "aimrt_module_c_interface/module_base.h"
#include "aimrt_module_cpp_interface/core.h"
#include "aimrt_module_cpp_interface/util/string.h"

namespace aimrt {

/**
 * @brief All modules need inherit this interface class
 *
 */
class ModuleBase {
 public:
  ModuleBase() : base_(GenBase(this)) {}
  virtual ~ModuleBase() = default;

  ModuleBase(const ModuleBase&) = delete;
  ModuleBase& operator=(const ModuleBase&) = delete;

  /**
   * @brief Get module info
   *
   * @return module info
   */
  virtual ModuleInfo Info() const = 0;

  /**
   * @brief Initialize module
   *
   * @param core Abstract of framework
   * @return initialize result
   */
  virtual bool Initialize(CoreRef core) = 0;

  /**
   * @brief Start module
   *
   * @return Start result
   */
  virtual bool Start() = 0;

  /**
   * @brief Shutdown module
   *
   */
  virtual void Shutdown() = 0;

  /**
   * @brief Get native handle
   *
   * @return Native handle
   */
  const aimrt_module_base_t* NativeHandle() const { return &base_; }

 private:
  static aimrt_module_base_t GenBase(void* impl) {
    return aimrt_module_base_t{
        .info = [](void* impl) -> aimrt_module_info_t {
          try {
            auto module_info = static_cast<ModuleBase*>(impl)->Info();
            return aimrt_module_info_t{
                .name = aimrt::util::ToAimRTStringView(module_info.name),
                .major_version = module_info.major_version,
                .minor_version = module_info.minor_version,
                .patch_version = module_info.patch_version,
                .build_version = module_info.build_version,
                .author = aimrt::util::ToAimRTStringView(module_info.author),
                .description = aimrt::util::ToAimRTStringView(module_info.description)};
          } catch (...) {
            return aimrt_module_info_t{};
          }
        },
        .initialize = [](void* impl, const aimrt_core_base_t* core) -> bool {
          try {
            return static_cast<ModuleBase*>(impl)->Initialize(CoreRef(core));
          } catch (...) {
            return false;
          }
        },
        .start = [](void* impl) -> bool {
          try {
            return static_cast<ModuleBase*>(impl)->Start();
          } catch (...) {
            return false;
          }
        },
        .shutdown = [](void* impl) {
          try {
            static_cast<ModuleBase*>(impl)->Shutdown();
          } catch (...) {
          }  //
        },
        .impl = impl};
  }

 private:
  const aimrt_module_base_t base_;
};

}  // namespace aimrt
