// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/plugin/plugin_loader.h"

namespace aimrt::runtime::core::plugin {

using DynlibCreateCorePluginFunc = aimrt::AimRTCorePluginBase* (*)();
using DynlibDestroyCorePluginFunc = void (*)(const aimrt::AimRTCorePluginBase*);

static constexpr const char* kDynlibCreateCorePluginFuncName = "AimRTDynlibCreateCorePluginHandle";
static constexpr const char* kDynlibDestroyCorePluginFuncName = "AimRTDynlibDestroyCorePluginHandle";

void PluginLoader::LoadPlugin(std::string_view plugin_path) {
  plugin_path_ = plugin_path;

  AIMRT_CHECK_ERROR_THROW(dynamic_lib_.Load(plugin_path_),
                          "Load dynamic lib failed, lib path {}, error info {}",
                          plugin_path_, aimrt::common::util::DynamicLib::GetErr());

  auto* create_func = dynamic_lib_.GetSymbol(kDynlibCreateCorePluginFuncName);
  AIMRT_CHECK_ERROR_THROW(create_func != nullptr,
                          "Cannot find symbol '{}' in lib {}.",
                          kDynlibCreateCorePluginFuncName, plugin_path_);

  destroy_func_ = dynamic_lib_.GetSymbol(kDynlibDestroyCorePluginFuncName);
  AIMRT_CHECK_ERROR_THROW(destroy_func_ != nullptr,
                          "Cannot find symbol '{}' in lib {}.",
                          kDynlibDestroyCorePluginFuncName, plugin_path_);

  // 加载plugin
  plugin_ptr_ = ((DynlibCreateCorePluginFunc)create_func)();

  AIMRT_CHECK_ERROR_THROW(plugin_ptr_ != nullptr,
                          "Cannot create plugin in lib {}.", plugin_path_);
}

void PluginLoader::UnLoadPlugin() {
  if (!dynamic_lib_.IsLoaded()) return;

  if (plugin_ptr_ != nullptr) {
    if (destroy_func_ != nullptr) {
      ((DynlibDestroyCorePluginFunc)destroy_func_)(plugin_ptr_);
    } else {
      AIMRT_WARN("Destroy func is null!");
    }
  }

  destroy_func_ = nullptr;
  dynamic_lib_.Unload();
}

}  // namespace aimrt::runtime::core::plugin
