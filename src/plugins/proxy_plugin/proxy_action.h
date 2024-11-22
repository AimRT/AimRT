// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_cpp_interface/util/type_support.h"
#include "core/util/topic_meta_key.h"

#include "topic_meta.h"
#include "yaml-cpp/yaml.h"

namespace aimrt::plugins::proxy_plugin {

class ProxyAction {
 public:
  struct Options {
    struct TopicMeta {
      std::string sub_topic_name;
      std::vector<std::string> pub_topic_name;
      std::string msg_type;
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

  std::unordered_map<runtime::core::util::TopicMetaKey, TopicMeta,
                     runtime::core::util::TopicMetaKey::Hash>
      topic_meta_map_;
  std::unordered_set<runtime::core::util::TopicMetaKey,
                     runtime::core::util::TopicMetaKey::Hash>
      pub_topic_name_set_;
};

}  // namespace aimrt::plugins::proxy_plugin
