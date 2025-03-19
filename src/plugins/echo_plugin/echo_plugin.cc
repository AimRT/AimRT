// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "echo_plugin/echo_plugin.h"

#include <string>

#include "core/aimrt_core.h"
#include "core/channel/channel_backend_tools.h"
#include "echo_plugin/global.h"

#include "json/json.h"
#include "yaml-cpp/yaml.h"

namespace YAML {

template <>
struct convert<aimrt::plugins::echo_plugin::EchoPlugin::Options> {
  using Options = aimrt::plugins::echo_plugin::EchoPlugin::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["type_support_pkgs"] = Node(NodeType::Sequence);
    for (const auto& type_support_pkg : rhs.type_support_pkgs) {
      Node type_support_pkg_node;
      type_support_pkg_node["path"] = type_support_pkg.path;
      node["type_support_pkgs"].push_back(type_support_pkg_node);
    }

    node["topic_meta_list"] = Node(NodeType::Sequence);

    for (const auto& topic_meta : rhs.topic_meta_list) {
      Node topic_meta_node;
      topic_meta_node["topic_name"] = topic_meta.topic_name;
      topic_meta_node["msg_type"] = topic_meta.msg_type;
      topic_meta_node["echo_type"] = topic_meta.echo_type;
      node["topic_meta_list"].push_back(topic_meta_node);
    }
    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["type_support_pkgs"] && node["type_support_pkgs"].IsSequence()) {
      for (const auto& type_support_pkg_node : node["type_support_pkgs"]) {
        Options::TypeSupportPkg type_support_pkg;
        type_support_pkg.path = type_support_pkg_node["path"].as<std::string>();
        rhs.type_support_pkgs.emplace_back(std::move(type_support_pkg));
      }
    }

    if (node["topic_meta_list"] && node["topic_meta_list"].IsSequence()) {
      for (const auto& topic_meta_node : node["topic_meta_list"]) {
        Options::TopicMeta topic_meta;
        topic_meta.topic_name = topic_meta_node["topic_name"].as<std::string>();
        topic_meta.msg_type = topic_meta_node["msg_type"].as<std::string>();
        if (topic_meta_node["echo_type"] && topic_meta_node["echo_type"].IsScalar()) {
          topic_meta.echo_type = topic_meta_node["echo_type"].as<std::string>();
        } else {
          topic_meta.echo_type = "json";
        }
        rhs.topic_meta_list.push_back(std::move(topic_meta));
      }
    }
    return true;
  }
};

}  // namespace YAML

namespace aimrt::plugins::echo_plugin {

bool EchoPlugin::Initialize(runtime::core::AimRTCore* core_ptr) noexcept {
  try {
    core_ptr_ = core_ptr;

    YAML::Node plugin_options_node = core_ptr_->GetPluginManager().GetPluginOptionsNode(Name());

    if (plugin_options_node && !plugin_options_node.IsNull()) {
      options_ = plugin_options_node.as<Options>();
      AIMRT_TRACE("Load plugin options.");
    }

    init_flag_ = true;
    // type support
    for (auto& type_support_pkg : options_.type_support_pkgs) {
      // check duplicate pkg
      auto finditr = std::find_if(
          options_.type_support_pkgs.begin(), options_.type_support_pkgs.end(),
          [&type_support_pkg](const auto& op) {
            if (&type_support_pkg == &op) return false;
            return op.path == type_support_pkg.path;
          });
      AIMRT_CHECK_ERROR_THROW(finditr == options_.type_support_pkgs.end(),
                              "Duplicate pkg path {}", type_support_pkg.path);

      InitTypeSupport(type_support_pkg);
    }
    AIMRT_TRACE("Load {} pkg and {} type.",
                type_support_pkg_loader_vec_.size(), type_support_map_.size());

    for (auto& topic_meta : options_.topic_meta_list) {
      // check msg type
      auto finditr = type_support_map_.find(topic_meta.msg_type);
      AIMRT_CHECK_ERROR_THROW(finditr != type_support_map_.end(),
                              "Can not find type '{}' in any type support pkg!", topic_meta.msg_type);
    }

    // check duplicate topic
    for (auto& topic_meta_option : options_.topic_meta_list) {
      runtime::core::util::TopicMetaKey key{
          .topic_name = topic_meta_option.topic_name,
          .msg_type = topic_meta_option.msg_type};

      AIMRT_CHECK_ERROR_THROW(
          topic_meta_map_.find(key) == topic_meta_map_.end(),
          "Duplicate topic meta, topic name: {}, msg type: {}.",
          topic_meta_option.topic_name, topic_meta_option.msg_type);

      TopicMeta topic_meta{
          .topic_name = topic_meta_option.topic_name,
          .msg_type = topic_meta_option.msg_type,
          .echo_type = topic_meta_option.echo_type,
      };
      topic_meta_map_.emplace(key, topic_meta);
    }

    core_ptr_->RegisterHookFunc(
        runtime::core::AimRTCore::State::kPostInitLog,
        [this] {
          SetLogger(aimrt::logger::LoggerRef(
              core_ptr_->GetLoggerManager().GetLoggerProxy().NativeHandle()));
        });
    core_ptr_->RegisterHookFunc(
        runtime::core::AimRTCore::State::kPreInitModules,
        [this] {
          RegisterEchoChannel();
        });
    core_ptr_->RegisterHookFunc(
        runtime::core::AimRTCore::State::kPreShutdown,
        [this] {
          SetLogger(aimrt::logger::GetSimpleLoggerRef());
        });

    plugin_options_node = options_;
    core_ptr_->GetPluginManager().UpdatePluginOptionsNode(Name(), plugin_options_node);

    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Initialize failed, {}", e.what());
  }

  return false;
}

void EchoPlugin::InitTypeSupport(Options::TypeSupportPkg& options) {
  auto loader_ptr = std::make_unique<aimrt::runtime::core::util::TypeSupportPkgLoader>();
  loader_ptr->LoadTypeSupportPkg(options.path);

  options.path = loader_ptr->GetDynamicLib().GetLibFullPath();

  auto type_support_array = loader_ptr->GetTypeSupportArray();
  for (const auto* item : type_support_array) {
    aimrt::util::TypeSupportRef type_support_ref(item);
    auto type_name = type_support_ref.TypeName();

    // check duplicate type
    auto finditr = type_support_map_.find(type_name);
    if (finditr != type_support_map_.end()) {
      AIMRT_WARN("Duplicate msg type '{}' in {} and {}.",
                 type_name, options.path, finditr->second.options.path);
      continue;
    }

    type_support_map_.emplace(
        type_name,
        TypeSupportWrapper{
            .options = options,
            .type_support_ref = type_support_ref,
            .loader_ptr = loader_ptr.get()});
  }
  type_support_pkg_loader_vec_.emplace_back(std::move(loader_ptr));
}

void EchoPlugin::RegisterEchoChannel() {
  using namespace aimrt::runtime::core::channel;

  const auto& topic_meta_list = topic_meta_map_;
  AIMRT_TRACE("Echo plugin has {} topics.", topic_meta_list.size());

  for (const auto& topic_meta_itr : topic_meta_list) {
    const auto& topic_meta = topic_meta_itr.second;

    auto finditr = type_support_map_.find(topic_meta.msg_type);
    AIMRT_CHECK_ERROR_THROW(finditr != type_support_map_.end(),
                            "Can not find type '{}' in any type support pkg!", topic_meta.msg_type);

    const auto& type_support_wrapper = finditr->second;

    SubscribeWrapper sub_wrapper;
    sub_wrapper.info = TopicInfo{
        .msg_type = topic_meta.msg_type,
        .topic_name = topic_meta.topic_name,
        .pkg_path = type_support_wrapper.options.path,
        .module_name = "core",
        .msg_type_support_ref = type_support_wrapper.type_support_ref};

    sub_wrapper.require_cache_serialization_types.emplace(topic_meta.echo_type);
    sub_wrapper.callback = [echo_type{topic_meta.echo_type}, this](
                               MsgWrapper& msg_wrapper, std::function<void()>&& release_callback) {
      auto buffer_view_ptr = aimrt::runtime::core::channel::TrySerializeMsgWithCache(msg_wrapper, echo_type);
      if (!buffer_view_ptr) [[unlikely]] {
        AIMRT_ERROR("Can not serialize msg type '{}' with echo type '{}'.",
                    msg_wrapper.info.msg_type, echo_type);
        release_callback();
        return;
      }
      std::string echo_str;
      if (buffer_view_ptr->Size() == 1) {
        auto data = buffer_view_ptr->Data()[0];
        echo_str = std::string(static_cast<const char*>(data.data), data.len);
      } else if (buffer_view_ptr->Size() > 1) {
        echo_str = buffer_view_ptr->JoinToString();
      } else {
        AIMRT_ERROR("Invalid buffer, topic_name: {}, msg_type: {}", msg_wrapper.info.topic_name, msg_wrapper.info.msg_type);
      }
      if (echo_type == "json") {
        FormatJson(echo_str);
      }
      if(!echo_str.empty())
        AIMRT_INFO("\n{}", echo_str);
      release_callback();
    };

    bool ret = core_ptr_->GetChannelManager().Subscribe(std::move(sub_wrapper));
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed!");
  }
}

void EchoPlugin::FormatJson(std::string& json_str) {
  Json::Value root;
  bool success = Json::Reader().parse(json_str, root);
  if (success) {
    json_str = Json::StyledWriter().write(root);
  } else {
    AIMRT_WARN("Invalid json string, will keep original string.");
  }
}

void EchoPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

}  // namespace aimrt::plugins::echo_plugin
