// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "aimrt_module_c_interface/module_base.h"
#include "core/util/dynamic_lib.h"
#include "util/log_util.h"
#include "util/string_util.h"

namespace aimrt::runtime::core::module {

class ModuleLoader {
 public:
  ModuleLoader()
      : logger_ptr_(std::make_shared<aimrt::common::util::LoggerWrapper>()) {}
  ~ModuleLoader() { UnLoadPkg(); }

  ModuleLoader(const ModuleLoader&) = delete;
  ModuleLoader& operator=(const ModuleLoader&) = delete;

  void SetLogger(const std::shared_ptr<aimrt::common::util::LoggerWrapper>& logger_ptr) { logger_ptr_ = logger_ptr; }
  const aimrt::common::util::LoggerWrapper& GetLogger() const { return *logger_ptr_; }

  void LoadPkg(std::string_view pkg_path,
               const std::vector<std::string>& disable_modules,
               const std::vector<std::string>& enable_modules);

  void UnLoadPkg();

  const std::vector<std::string>& GetModuleNameList() const {
    return module_name_vec_;
  }

  const std::vector<std::string>& GetLoadedModuleNameList() const {
    return loaded_module_name_vec_;
  }

  const aimrt_module_base_t* GetModule(std::string_view module_name);
  void DestroyModule(const aimrt_module_base_t* module_ptr);

  auto& GetDynamicLib() { return dynamic_lib_; }

 private:
  std::shared_ptr<aimrt::common::util::LoggerWrapper> logger_ptr_;

  std::string pkg_path_;
  aimrt::common::util::DynamicLib dynamic_lib_;

  aimrt::common::util::DynamicLib::SymbolType destroy_func_ = nullptr;

  std::vector<std::string> module_name_vec_;
  std::vector<std::string> loaded_module_name_vec_;
  std::unordered_map<
      std::string,
      const aimrt_module_base_t*,
      aimrt::common::util::StringHash,
      std::equal_to<>>
      module_ptr_map_;

  // Create an enumeration class to identify the usage of the pkgs module (UseNone: use all, UseEnable: use enabled module, UseDisable: use disabled module, use UseNone by default)
  enum class Enable_or_Disable : uint32_t {
    kUseNone,
    kUseEnable,
    kUseDisable,
  } enable_or_disable_for_pkg_ = Enable_or_Disable::kUseNone;
};

}  // namespace aimrt::runtime::core::module
