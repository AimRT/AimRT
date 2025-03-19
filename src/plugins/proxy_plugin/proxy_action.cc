// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "proxy_plugin/proxy_action.h"

#include "proxy_plugin/global.h"

namespace YAML {
template <>
struct convert<aimrt::plugins::proxy_plugin::ProxyAction::Options> {
  using Options = aimrt::plugins::proxy_plugin::ProxyAction::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["executor"] = rhs.executor;

    node["topic_meta_list"] = YAML::Node();
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

    rhs.executor = node["executor"].as<std::string>();

    if (node["topic_meta_list"]) {
      const auto& topic_meta_list = node["topic_meta_list"];
      if (topic_meta_list.IsSequence()) {
        for (const auto& topic_meta_node : topic_meta_list) {
          Options::TopicMeta topic_meta;
          topic_meta.sub_topic_name = topic_meta_node["sub_topic_name"].as<std::string>();
          topic_meta.pub_topic_name = topic_meta_node["pub_topic_name"].as<std::vector<std::string>>();
          topic_meta.msg_type = topic_meta_node["msg_type"].as<std::string>();
          rhs.topic_meta_list.push_back(topic_meta);
        }
      }
    }
    return true;
  }
};

}  // namespace YAML

namespace aimrt::plugins::proxy_plugin {
void ProxyAction::Initialize(YAML::Node options) {
  if (options && !options.IsNull())
    options_ = options.as<Options>();

  for (auto& topic_meta : options_.topic_meta_list) {
    // check msg type
    auto type_support_ref = get_type_support_func_(topic_meta.msg_type);
    AIMRT_CHECK_ERROR_THROW(type_support_ref, "Can not get type support for msg type '{}'.", topic_meta.msg_type);

    // check duplicate topic meta
    runtime::core::util::TopicMetaKey key{
        .topic_name = topic_meta.sub_topic_name,
        .msg_type = topic_meta.msg_type};

    AIMRT_CHECK_ERROR_THROW(
        topic_meta_map_.find(key) == topic_meta_map_.end(),
        "Duplicate topic meta, topic name: {}, msg type: {}.",
        topic_meta.sub_topic_name, topic_meta.msg_type);

    TopicMeta topic_meta_info{
        .topic_name = topic_meta.sub_topic_name,
        .msg_type = topic_meta.msg_type,
        .pub_topic_name = topic_meta.pub_topic_name};
    topic_meta_map_.emplace(key, topic_meta_info);

    // check duplicate pub topic name
    for (const auto& pub_topic_name : topic_meta.pub_topic_name) {
      runtime::core::util::TopicMetaKey pub_key{
          .topic_name = pub_topic_name,
          .msg_type = topic_meta.msg_type};
      AIMRT_CHECK_ERROR_THROW(
          pub_topic_name_set_.find(pub_key) == pub_topic_name_set_.end(),
          "Duplicate pub topic name: {}, msg type: {}.",
          pub_topic_name, topic_meta.msg_type);
      pub_topic_name_set_.insert(pub_key);
    }
  }
}

void ProxyAction::InitExecutor() {
  executor_ = get_executor_func_(options_.executor);

  AIMRT_CHECK_ERROR_THROW(executor_, "Can not get executor {}.", options_.executor);

  AIMRT_CHECK_ERROR_THROW(
      executor_.ThreadSafe(),
      "Proxy executor {} is not thread safe!", options_.executor);
}

void ProxyAction::RegisterGetExecutorFunc(
    const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func) {
  get_executor_func_ = get_executor_func;
}

void ProxyAction::RegisterGetTypeSupportFunc(
    const std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func) {
  get_type_support_func_ = get_type_support_func;
}

void ProxyAction::Shutdown() {
  AIMRT_INFO("Proxy action shutdown.");
}

}  // namespace aimrt::plugins::proxy_plugin