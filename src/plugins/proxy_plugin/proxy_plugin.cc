// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "proxy_plugin.h"
#include "aimrt_core.h"
#include "aimrt_core_plugin_base.h"
#include "channel/channel_backend_tools.h"
#include "channel/channel_msg_wrapper.h"
#include "channel/channel_registry.h"
#include "global.h"
#include "log_util.h"
#include "proxy_action.h"
#include "proxy_plugin.h"
#include "proxy_plugin/topic_meta.h"

namespace YAML {

template <>
struct convert<aimrt::plugins::proxy_plugin::ProxyPlugin::Options> {
  using Options = aimrt::plugins::proxy_plugin::ProxyPlugin::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["type_support_pkgs"] = Node(NodeType::Sequence);
    for (const auto& type_support_pkg : rhs.type_support_pkgs) {
      Node type_support_pkg_node;
      type_support_pkg_node["path"] = type_support_pkg.path;
      node["type_support_pkgs"].push_back(type_support_pkg_node);
    }

    node["executor"] = rhs.executor;
    node["proxy_actions"] = Node(NodeType::Sequence);

    for (const auto& proxy_action : rhs.proxy_actions) {
      Node proxy_action_node;
      proxy_action_node["name"] = proxy_action.name;
      proxy_action_node["options"] = proxy_action.options;
      node["proxy_actions"].push_back(proxy_action_node);
    }
    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["type_support_pkgs"] && node["type_support_pkgs"].IsSequence()) {
      for (const auto& type_support_pkg_node : node["type_support_pkgs"]) {
        Options::TypeSupportPkg type_support_pkg;
        type_support_pkg.path = type_support_pkg_node["path"].as<std::string>();
        rhs.type_support_pkgs.push_back(std::move(type_support_pkg));
      }
    }

    if (node["executor"] && node["executor"].IsScalar()) {
      rhs.executor = node["executor"].as<std::string>();
    }

    if (node["proxy_actions"] && node["proxy_actions"].IsSequence()) {
      for (const auto& proxy_action_node : node["proxy_actions"]) {
        Options::ProxyAction proxy_action;
        proxy_action.name = proxy_action_node["name"].as<std::string>();
        proxy_action.options = proxy_action_node["options"];
        rhs.proxy_actions.push_back(std::move(proxy_action));
      }
    }
    return true;
  }
};

}  // namespace YAML

namespace aimrt::plugins::proxy_plugin {

bool ProxyPlugin::Initialize(runtime::core::AimRTCore* core_ptr) noexcept {
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

    // proxy action
    for (auto& proxy_action : options_.proxy_actions) {
      // check duplicate proxy action name
      auto finditr = std::find_if(
          options_.proxy_actions.begin(), options_.proxy_actions.end(),
          [&proxy_action](const auto& op) {
            if (&proxy_action == &op) return false;
            return op.name == proxy_action.name;
          });
      AIMRT_CHECK_ERROR_THROW(finditr == options_.proxy_actions.end(),
                              "Duplicate proxy action name {}", proxy_action.name);

      auto action_ptr = std::make_unique<ProxyAction>();

      action_ptr->RegisterGetExecutorFunc([this](std::string_view name) -> aimrt::executor::ExecutorRef {
        return core_ptr_->GetExecutorManager().GetExecutor(name);
      });
      action_ptr->RegisterGetTypeSupportFunc([this](std::string_view name) -> aimrt::util::TypeSupportRef {
        auto finditr = type_support_map_.find(name);
        AIMRT_CHECK_ERROR_THROW(finditr != type_support_map_.end(),
                                "Can not find type support for msg type '{}'.", name);
        return finditr->second.type_support_ref;
      });
      action_ptr->Initialize(proxy_action.options);

      proxy_action_map_.emplace(proxy_action.name, std::move(action_ptr));
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
          for (auto& proxy_action_itr : proxy_action_map_) {
            proxy_action_itr.second->InitExecutor();
          }
          RegisterSubChannel();
          RegisterPubChannel();
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

void ProxyPlugin::InitTypeSupport(Options::TypeSupportPkg& options) {
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
  AIMRT_TRACE("Load {} type support pkgs.", type_support_pkg_loader_vec_.size());
}

void ProxyPlugin::RegisterSubChannel() {
  using namespace aimrt::runtime::core::channel;

  for (const auto& [_, proxy_action] : proxy_action_map_) {
    for (const auto& [_, topic_meta] : proxy_action->GetTopicMetaMap()) {
      auto finditr = type_support_map_.find(topic_meta.msg_type);
      AIMRT_CHECK_ERROR_THROW(finditr != type_support_map_.end(),
                              "Can not find type '{}' in any type support pkg!", topic_meta.msg_type);

      const auto& type_support_wrapper = finditr->second;

      SubscribeWrapper subscribe_wrapper{
          .info = {
              .msg_type = topic_meta.msg_type,
              .topic_name = topic_meta.topic_name,
              .pkg_path = type_support_wrapper.options.path,
              .module_name = "core",
              .msg_type_support_ref = type_support_wrapper.type_support_ref}};
      subscribe_wrapper.require_cache_serialization_types.emplace(topic_meta.serialization_type);

      subscribe_wrapper.callback = [this, &proxy_action, serialization_type{topic_meta.serialization_type}](
                                       MsgWrapper& msg_wrapper, std::function<void()>&& release_callback) {
        auto buffer_view_ptr = aimrt::runtime::core::channel::TrySerializeMsgWithCache(msg_wrapper, serialization_type);
        if (!buffer_view_ptr) [[unlikely]] {
          AIMRT_WARN("Can not serialize msg type '{}' with serialization type '{}'.",
                     msg_wrapper.info.msg_type, serialization_type);
          release_callback();
          return;
        }
        proxy_action->GetExecutor().Execute([this, msg_wrapper, topic_meta_map = proxy_action->GetTopicMetaMap()]() {

          runtime::core::util::TopicMetaKey key{
            .topic_name = msg_wrapper.info.topic_name,
            .msg_type = msg_wrapper.info.msg_type,
          };
          
          auto finditr = topic_meta_map.find(key);
          AIMRT_CHECK_ERROR_THROW(finditr != topic_meta_map.end(),
                                  "Can not find topic meta, topic name: {}, msg type: {}.",
                                  key.topic_name, key.msg_type);
          
          for (auto &pub_topic_name : finditr->second.pub_topic_name) {
            runtime::core::util::TopicMetaKey pub_key{
              .topic_name = pub_topic_name,
              .msg_type = key.msg_type,
            };
            const auto& topic_pub_wrapper = topic_pub_wrapper_map_.find(pub_key)->second;
            AIMRT_CHECK_ERROR_THROW(topic_pub_wrapper.pub_type_wrapper_ptr, "Get publish type wrapper failed!");
            
            aimrt::channel::Context ctx;  
            MsgWrapper pub_msg_wrapper{
              .info = topic_pub_wrapper.pub_type_wrapper_ptr->info,
              .msg_ptr = nullptr,
              .ctx_ref = ctx,
            };
            pub_msg_wrapper.serialization_cache = msg_wrapper.serialization_cache;

            core_ptr_->GetChannelManager().Publish(std::move(pub_msg_wrapper));

          } });
        release_callback();
      };
      bool ret = core_ptr_->GetChannelManager().Subscribe(std::move(subscribe_wrapper));
      AIMRT_CHECK_ERROR_THROW(ret, "Register subscribe channel failed!");
    }
  }
}

void ProxyPlugin::RegisterPubChannel() {
  using namespace aimrt::runtime::core::channel;

  for (auto& proxy_action_itr : proxy_action_map_) {
    // register publish type
    auto& proxy_action = *(proxy_action_itr.second);

    const auto& topic_meta_map = proxy_action.GetTopicMetaMap();

    for (const auto& topic_meta_itr : topic_meta_map) {
      const auto& topic_meta = topic_meta_itr.second;

      auto finditr = type_support_map_.find(topic_meta.msg_type);
      AIMRT_CHECK_ERROR_THROW(finditr != type_support_map_.end(),
                              "Can not find type '{}' in any type support pkg!", topic_meta.msg_type);

      const auto& type_support_wrapper = finditr->second;

      // register publish type
      for (auto& pub_topic_name : topic_meta.pub_topic_name) {
        PublishTypeWrapper pub_type_wrapper;
        pub_type_wrapper.info = TopicInfo{
            .msg_type = topic_meta.msg_type,
            .topic_name = pub_topic_name,
            .pkg_path = type_support_wrapper.options.path,
            .module_name = "core",
            .msg_type_support_ref = type_support_wrapper.type_support_ref};

        bool ret = core_ptr_->GetChannelManager().RegisterPublishType(std::move(pub_type_wrapper));
        AIMRT_CHECK_ERROR_THROW(ret, "Register publish type failed!");
      }
    }
    // map pub_type_wrapper_ptr
    for (auto& topic_meta_itr : topic_meta_map) {
      const auto& topic_meta = topic_meta_itr.second;
      for (auto& pub_topic_name : topic_meta.pub_topic_name) {
        runtime::core::util::TopicMetaKey key{
            .topic_name = pub_topic_name,
            .msg_type = topic_meta.msg_type};
        auto finditr = type_support_map_.find(topic_meta.msg_type);
        AIMRT_CHECK_ERROR_THROW(finditr != type_support_map_.end(),
                                "Can not find type '{}' in any type support pkg!", topic_meta.msg_type);

        const auto& type_support_wrapper = finditr->second;

        const auto* pub_type_wrapper_ptr = core_ptr_->GetChannelManager().GetChannelRegistry()->GetPublishTypeWrapperPtr(
            topic_meta.msg_type, pub_topic_name, type_support_wrapper.options.path, "core");

        AIMRT_CHECK_ERROR_THROW(pub_type_wrapper_ptr, "Get publish type wrapper failed!");

        topic_pub_wrapper_map_.emplace(key, TopicPubWrapper{
                                                .pub_type_wrapper_ptr = pub_type_wrapper_ptr,
                                                .serialization_type = topic_meta.serialization_type});
      }
    }
  }
}

void ProxyPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;
    for (auto& proxy_action_itr : proxy_action_map_) {
      proxy_action_itr.second->Shutdown();
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

}  // namespace aimrt::plugins::proxy_plugin