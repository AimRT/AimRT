// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "echo_plugin/echo_plugin.h"

#include <json-c/json.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include "echo_plugin/global.h"

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

    if (!rhs.executor.empty()) {
      node["executor"] = rhs.executor;
    }

    node["topic_meta_list"] = Node(NodeType::Sequence);
    for (const auto& topic_meta : rhs.topic_meta_list) {
      Node topic_meta_node;
      topic_meta_node["topic_name"] = topic_meta.topic_name;
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
        topic_meta.topic_name = topic_meta_node["topic_name"].as<std::string>();
        topic_meta.msg_type = topic_meta_node["msg_type"].as<std::string>();
        rhs.topic_meta_list.push_back(std::move(topic_meta));
      }
    }
    return true;
  }
};
YAML::Node json_to_yaml(json_object* json) {
  YAML::Node result;

  if (json == nullptr) {
    return result;
  }

  switch (json_object_get_type(json)) {
    case json_type_null:
      result = YAML::Node(YAML::NodeType::Null);
      break;
    case json_type_boolean:
      result = YAML::Node(json_object_get_boolean(json));
      break;
    case json_type_double:
      result = YAML::Node(json_object_get_double(json));
      break;
    case json_type_int:
      result = YAML::Node(json_object_get_int64(json));
      break;
    case json_type_string:
      result = YAML::Node(json_object_get_string(json));
      break;
    case json_type_array: {
      int length = json_object_array_length(json);
      for (int i = 0; i < length; i++) {
        result.push_back(json_to_yaml(json_object_array_get_idx(json, i)));
      }
      break;
    }
    case json_type_object: {
      json_object_object_foreach(json, key, val) {
        result[std::string(key)] = json_to_yaml(val);
      }
      break;
    }
  }
  return result;
}

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

    RegisterGetTypeSupportFunc(
        [this](std::string_view msg_type) -> aimrt::util::TypeSupportRef {
          auto finditr = type_support_map_.find(msg_type);
          if (finditr != type_support_map_.end())
            return finditr->second.type_support_ref;
          return {};
        });

    for (auto& topic_meta : options_.topic_meta_list) {
      // check msg type
      auto type_support_ref = get_type_support_func_(topic_meta.msg_type);
      AIMRT_CHECK_ERROR_THROW(type_support_ref,

                              "Can not find type '{}' in any type support pkg!", topic_meta.msg_type);

      // check serialization type
      if (!topic_meta.serialization_type.empty()) {
        bool check_ret = type_support_ref.CheckSerializationTypeSupported(topic_meta.serialization_type);
        AIMRT_CHECK_ERROR_THROW(check_ret,
                                "Msg type '{}' does not support serialization type '{}'.",
                                topic_meta.msg_type, topic_meta.msg_type);
      } else {
        topic_meta.serialization_type = type_support_ref.DefaultSerializationType();
      }
    }

    // check duplicate topic
    for (auto& topic_meta_option : options_.topic_meta_list) {
      TopicMetaKey key{
          .topic_name = topic_meta_option.topic_name,
          .msg_type = topic_meta_option.msg_type};

      AIMRT_CHECK_ERROR_THROW(
          topic_meta_map_.find(key) == topic_meta_map_.end(),
          "Duplicate topic meta, topic name: {}, msg type: {}.",
          topic_meta_option.topic_name, topic_meta_option.msg_type);

      TopicMeta topic_meta{
          .topic_name = topic_meta_option.topic_name,
          .msg_type = topic_meta_option.msg_type,
          .serialization_type = topic_meta_option.serialization_type};
      topic_meta_map_.emplace(key, topic_meta);
    }

    if (!options_.executor.empty()) {
      core_ptr_->RegisterHookFunc(
          runtime::core::AimRTCore::State::kPostStart,
          [this] {
            RegisterGetExecutorFunc(
                [this](std::string_view executor_name) -> aimrt::executor::ExecutorRef {
                  return core_ptr_->GetExecutorManager().GetExecutor(executor_name);
                });
            InitExecutor();
          });
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
          Shutdown();
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
  auto loader_ptr = std::make_unique<TypeSupportPkgLoader>();
  loader_ptr->LoadTypeSupportPkg(options.path);

  options.path = loader_ptr->GetDynamicLib().GetLibFullPath();

  auto type_support_array = loader_ptr->GetTypeSupportArray();

  for (const auto* item : type_support_array) {
    aimrt::util::TypeSupportRef type_support_ref(item);
    auto type_name = type_support_ref.TypeName();

    // 检查重复 type
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
  using EchoFunc = std::function<void(uint64_t, MsgWrapper&)>;

  struct Wrapper {
    std::unordered_set<std::string> require_cache_serialization_types;
    std::vector<EchoFunc> echo_func_vec;
  };

  std::unordered_map<TopicMetaKey, Wrapper, TopicMetaKey::Hash> echo_func_map;

  const auto& topic_meta_list = topic_meta_map_;
  AIMRT_TRACE("Echo plugin has {} topics.", topic_meta_list.size());

  for (const auto& topic_meta_itr : topic_meta_list) {
    const auto& topic_meta = topic_meta_itr.second;
    EchoFunc echo_func;
    if (options_.executor.empty()) {
      echo_func = [this, serialization_type{topic_meta.serialization_type}](
                      uint64_t cur_timestamp, MsgWrapper& msg_wrapper) {
        Echo(msg_wrapper, serialization_type);
      };
    } else {
      echo_func = [this, serialization_type{topic_meta.serialization_type}](
                      uint64_t cur_timestamp, MsgWrapper& msg_wrapper) {
        executor_.Execute([this, msg_wrapper{std::move(msg_wrapper)}, serialization_type]() mutable {
          Echo(msg_wrapper, serialization_type);
        });
      };
    }
    auto& item = echo_func_map[topic_meta_itr.first];
    item.require_cache_serialization_types.emplace(topic_meta.serialization_type);
    item.echo_func_vec.emplace_back(std::move(echo_func));
  }

  AIMRT_TRACE("Register {} echo functions.", echo_func_map.size());

  // subscribe
  for (auto& echo_func_itr : echo_func_map) {
    const auto& key = echo_func_itr.first;
    auto& wrapper = echo_func_itr.second;
    auto finditr = type_support_map_.find(key.msg_type);

    const auto& type_support_ref = finditr->second;

    SubscribeWrapper sub_wrapper;
    sub_wrapper.info = TopicInfo{
        .msg_type = key.msg_type,
        .topic_name = key.topic_name,
        .pkg_path = type_support_ref.options.path,
        .module_name = "core",
        .msg_type_support_ref = type_support_ref.type_support_ref};

    sub_wrapper.require_cache_serialization_types = wrapper.require_cache_serialization_types;

    sub_wrapper.callback = [echo_func_vec{std::move(wrapper.echo_func_vec)}](
                               MsgWrapper& msg_wrapper, std::function<void()>&& release_callback) {
      auto cur_timestamp = aimrt::common::util::GetCurTimestampNs();

      for (const auto& echo_func : echo_func_vec)
        echo_func(cur_timestamp, msg_wrapper);

      release_callback();
    };

    bool ret = core_ptr_->GetChannelManager().Subscribe(std::move(sub_wrapper));
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed!");
  }
}

void EchoPlugin::Shutdown() noexcept {
  try {
    if (!init_flag_) return;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
  }
}

void EchoPlugin::Echo(runtime::core::channel::MsgWrapper& msg_wrapper, std::string_view serialization_type) {
  auto buffer_view_ptr = aimrt::runtime::core::channel::TrySerializeMsgWithCache(msg_wrapper, "json");
  if (!buffer_view_ptr) [[unlikely]] {
    AIMRT_ERROR("Can not serialize msg type '{}' with serialization type '{}'.",
                msg_wrapper.info.msg_type, serialization_type);
    return;
  }
  if (buffer_view_ptr && buffer_view_ptr->Data() && buffer_view_ptr->Size() > 0) {
    const char* data = static_cast<const char*>(buffer_view_ptr->Data()[0].data);
    size_t size = buffer_view_ptr->Data()[0].len;
    std::string msg_content(data, size);

    json_object* jobj = json_tokener_parse(msg_content.c_str());
    if (jobj) {
      YAML::Node yaml_node = YAML::json_to_yaml(jobj);
      std::cout << yaml_node << "\n--------------------------------\n";
      json_object_put(jobj);  // release json object

    } else {
      AIMRT_ERROR("JSON SERIALIZATION ERROR, original message content: {}", msg_content);
    }
  } else {
    AIMRT_ERROR("Invalid buffer, topic_name: {}, msg_type: {}", msg_wrapper.info.topic_name, msg_wrapper.info.msg_type);
  }
}

void EchoPlugin::RegisterGetTypeSupportFunc(
    const std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func) {
  get_type_support_func_ = get_type_support_func;
}

void EchoPlugin::InitExecutor() {
  AIMRT_CHECK_ERROR_THROW(
      get_executor_func_,
      "Get executor function is not set before initialize.");

  executor_ = get_executor_func_(options_.executor);

  AIMRT_CHECK_ERROR_THROW(
      executor_, "Can not get executor {}.", options_.executor);
}

void EchoPlugin::RegisterGetExecutorFunc(
    const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func) {
  get_executor_func_ = get_executor_func;
}
}  // namespace aimrt::plugins::echo_plugin
