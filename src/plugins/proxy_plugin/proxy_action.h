// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/util/type_support.h"
#include "channel/channel_registry.h"
#include "topic_meta_key.h"
#include "yaml-cpp/yaml.h"

namespace aimrt::plugins::proxy_plugin {

class ProxyAction {
 public:
  struct Options {
    struct TopicMeta {
      std::string sub_topic_name;
      std::vector<std::string> pub_topic_name;
      std::string msg_type;
      std::string serialization_type;
    };
    std::vector<TopicMeta> topic_meta_list;
    struct TypeSupportPkg {
      std::string path;
    };
    std::vector<TypeSupportPkg> type_support_pkgs;
    std::string executor;
  };

 public:
  ProxyAction() = default;
  ~ProxyAction() = default;

  void Initialize(YAML::Node options);
  void InitExecutor();
  void Shutdown();

  const auto& GetTopicMetaMap() const {
    return topic_meta_map_;
  }

  auto& GetExecutor() { return executor_; }

  void RegisterGetExecutorFunc(
      const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func);

  void RegisterGetTypeSupportFunc(
      const std::function<aimrt::util::TypeSupportRef(std::string_view)>& get_type_support_func);

 private:
  Options options_;

  aimrt::executor::ExecutorRef executor_;

  std::function<executor::ExecutorRef(std::string_view)> get_executor_func_;
  std::function<aimrt::util::TypeSupportRef(std::string_view)> get_type_support_func_;

  std::unordered_map<TopicMetaKey, TopicMeta, TopicMetaKey::Hash> topic_meta_map_;
  std::unordered_set<TopicMetaKey, TopicMetaKey::Hash> pub_topic_name_set_;
};

}  // namespace aimrt::plugins::proxy_plugin
