// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "proxy_plugin/proxy_plugin.h"
#include "aimrt_core.h"
#include "aimrt_core_plugin_base.h"
#include "channel/channel_msg_wrapper.h"
#include "channel/channel_backend_tools.h"
#include "log_util.h"
#include "proxy_plugin/proxy_plugin.h"
#include "global.h"
#include "proxy_plugin/topic_meta_key.h"

#include <yaml-cpp/yaml.h>
#include <cstdio>
#include <string>
#include <vector>


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
    node["topic_meta_list"] = Node(NodeType::Sequence);
    

    for (const auto& topic_meta : rhs.topic_meta_list) {
      Node topic_meta_node;
      topic_meta_node["sub_topic_name"] = topic_meta.sub_topic_name;
      topic_meta_node["pub_topic_name"] = topic_meta.pub_topic_name;
      topic_meta_node["msg_type"] = topic_meta.msg_type;
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
        rhs.type_support_pkgs.push_back(std::move(type_support_pkg));
      }
    }
    
    if (node["executor"] && node["executor"].IsScalar()) {
      rhs.executor = node["executor"].as<std::string>();
    }

    if (node["topic_meta_list"] && node["topic_meta_list"].IsSequence()) {
      for (const auto& topic_meta_node : node["topic_meta_list"]) {
        Options::TopicMeta topic_meta;
        topic_meta.sub_topic_name = topic_meta_node["sub_topic_name"].as<std::string>();
        topic_meta.pub_topic_name = topic_meta_node["pub_topic_name"].as<std::vector<std::string>>();
        topic_meta.msg_type = topic_meta_node["msg_type"].as<std::string>();
        rhs.topic_meta_list.push_back(std::move(topic_meta));
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

    for (auto& topic_meta : options_.topic_meta_list) {
      // check msg type
      auto finditr = type_support_map_.find(topic_meta.msg_type);
      AIMRT_CHECK_ERROR_THROW(finditr != type_support_map_.end(),
                              "Can not find type '{}' in any type support pkg!", topic_meta.msg_type);
      auto& type_support_ref = finditr->second.type_support_ref;

      // check serialization type
      if (!topic_meta.serialization_type.empty()) {
        bool check_ret = type_support_ref.CheckSerializationTypeSupported(topic_meta.serialization_type);
        AIMRT_CHECK_ERROR_THROW(check_ret,
                                "Msg type '{}' does not support serialization type '{}'.",
                                topic_meta.msg_type, topic_meta.serialization_type);
      } else {
        topic_meta.serialization_type = type_support_ref.DefaultSerializationType();
      }
    }

    // check duplicate topic
    for (auto& topic_meta_option : options_.topic_meta_list) {
      TopicMetaKey key{
          .topic_name = topic_meta_option.sub_topic_name,
          .msg_type = topic_meta_option.msg_type};
      AIMRT_CHECK_ERROR_THROW(
          topic_meta_map_.find(key) == topic_meta_map_.end(),
          "Duplicate topic meta, topic name: {}, msg type: {}.",
          topic_meta_option.sub_topic_name, topic_meta_option.msg_type);

      TopicMeta topic_meta{
          .topic_name = topic_meta_option.sub_topic_name,
          .msg_type = topic_meta_option.msg_type,
          .serialization_type = topic_meta_option.serialization_type,
          .pub_topic_name = topic_meta_option.pub_topic_name};
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
          RegisterSubChannel();
          RegisterPubChannel();
          executor_ = core_ptr_->GetExecutorManager().GetExecutor(options_.executor);
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
  AIMRT_INFO("Load {} type support pkgs.", type_support_pkg_loader_vec_.size());
  type_support_pkg_loader_vec_.emplace_back(std::move(loader_ptr));
}

void ProxyPlugin::RegisterSubChannel() {
  using namespace aimrt::runtime::core::channel;
  const auto& topic_meta_list = topic_meta_map_;

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

    sub_wrapper.require_cache_serialization_types.emplace(topic_meta.serialization_type);
    sub_wrapper.callback = [this, serialization_type{topic_meta.serialization_type}](
                               MsgWrapper& msg_wrapper, std::function<void()>&& release_callback) {
      auto buffer_view_ptr = aimrt::runtime::core::channel::TrySerializeMsgWithCache(
          msg_wrapper, serialization_type);
      if (!buffer_view_ptr) [[unlikely]] {
        AIMRT_WARN("Can not serialize msg type '{}' with serialization type '{}'.",
                   msg_wrapper.info.msg_type, serialization_type);
        release_callback();
        return;
      }
      executor_.Execute([this, msg_wrapper](){
        TopicMetaKey key{
          .topic_name = msg_wrapper.info.topic_name,
          .msg_type = msg_wrapper.info.msg_type,
        };
        
        auto finditr = topic_meta_map_.find(key);
        AIMRT_CHECK_ERROR_THROW(finditr != topic_meta_map_.end(),
                                "Can not find topic meta, topic name: {}, msg type: {}.",
                                key.topic_name, key.msg_type);
        
        for (auto &pub_topic_name : finditr->second.pub_topic_name) {
          TopicMetaKey pub_key{
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

        }
      });
      release_callback();
    };

    bool ret = core_ptr_->GetChannelManager().Subscribe(std::move(sub_wrapper));
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed!");
  }
}

void ProxyPlugin::RegisterPubChannel(){
  using namespace aimrt::runtime::core::channel;
  
  const auto& topic_meta_list = topic_meta_map_;

  for(const auto& topic_meta_itr : topic_meta_list){
    const auto& topic_meta = topic_meta_itr.second;

    auto finditr = type_support_map_.find(topic_meta.msg_type);
    AIMRT_CHECK_ERROR_THROW(finditr != type_support_map_.end(),
                            "Can not find type '{}' in any type support pkg!", topic_meta.msg_type);

    const auto& type_support_wrapper = finditr->second;
    
    // register publish type
    for (auto &pub_topic_name : topic_meta.pub_topic_name) {
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

  // map pub_type_wrapper_ptr
    for(auto& pub_topic_name : topic_meta.pub_topic_name){
      TopicMetaKey key{
        .topic_name = pub_topic_name,
        .msg_type = topic_meta.msg_type
      };
      auto finditr = type_support_map_.find(topic_meta.msg_type);
      AIMRT_CHECK_ERROR_THROW(finditr != type_support_map_.end(),
                              "Can not find type '{}' in any type support pkg!", topic_meta.msg_type);
                                
      const auto& type_support_wrapper = finditr->second;

      const auto* pub_type_wrapper_ptr = core_ptr_->GetChannelManager().GetChannelRegistry()->GetPublishTypeWrapperPtr(
              topic_meta.msg_type, pub_topic_name, type_support_wrapper.options.path, "core");

      AIMRT_CHECK_ERROR_THROW(pub_type_wrapper_ptr, "Get publish type wrapper failed!");

      topic_pub_wrapper_map_.emplace(key, TopicPubWrapper{
        .pub_type_wrapper_ptr = pub_type_wrapper_ptr,
        .serialization_type = topic_meta.serialization_type
      });
    }
  }
};

void ProxyPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;   
  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

}  // namespace aimrt::plugins::proxy_plugin
