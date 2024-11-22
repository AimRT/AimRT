// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>
#include <vector>

namespace aimrt::plugins::proxy_plugin {

struct TopicMeta {
  std::string topic_name;
  std::string msg_type;
  std::vector<std::string> pub_topic_name;
};

}  // namespace aimrt::plugins::proxy_plugin
