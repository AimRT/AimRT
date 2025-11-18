// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "service_introspection_plugin/service_introspection_plugin.h"
#include <iostream>

#include "core/aimrt_core.h"
#include "service_introspection_plugin/global.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::service_introspection_plugin::ServiceIntrospectionPlugin::Options> {
  using Options = aimrt::plugins::service_introspection_plugin::ServiceIntrospectionPlugin::Options;

  static Node encode(const Options &rhs) {
    Node node;

    return node;
  }

  static bool decode(const Node &node, Options &rhs) {
    if (!node.IsMap()) return false;

    return true;
  }
};
}  // namespace YAML

namespace aimrt::plugins::service_introspection_plugin {

bool ServiceIntrospectionPlugin::Initialize(runtime::core::AimRTCore *core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;

    YAML::Node plugin_options_node = core_ptr_->GetPluginManager().GetPluginOptionsNode(Name());

    if (plugin_options_node && !plugin_options_node.IsNull()) {
      options_ = plugin_options_node.as<Options>();
    }

    init_flag_ = true;

    core_ptr_->GetRpcManager().RegisterClientFilter("service_introspection", [](const std::shared_ptr<aimrt::runtime::core::rpc::InvokeWrapper> &wrapper_ptr,
                                                                                aimrt::runtime::core::rpc::FrameworkAsyncRpcHandle &&h) {
      h(wrapper_ptr);
    });

    plugin_options_node = options_;
    core_ptr_->GetPluginManager().UpdatePluginOptionsNode(Name(), plugin_options_node);

    return true;
  } catch (const std::exception &e) {
    AIMRT_ERROR("Initialize failed, {}", e.what());
  }

  return false;
}

void ServiceIntrospectionPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;

    stop_flag_ = true;

  } catch (const std::exception &e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

void ServiceIntrospectionPlugin::SetPluginLogger() {
  SetLogger(aimrt::logger::LoggerRef(
      core_ptr_->GetLoggerManager().GetLoggerProxy().NativeHandle()));
}

}  // namespace aimrt::plugins::service_introspection_plugin
