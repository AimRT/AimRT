// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>

namespace aimrt::plugins::echo_plugin {

struct TopicMeta {
  std::string topic_name;
  std::string msg_type;
  std::string echo_type;
};

}  // namespace aimrt::plugins::echo_plugin
