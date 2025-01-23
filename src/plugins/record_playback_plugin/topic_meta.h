// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>
#include <string>

namespace aimrt::plugins::record_playback_plugin {

struct TopicMeta {
  uint64_t id;
  std::string topic_name;
  std::string msg_type;
  std::string serialization_type;
};

}  // namespace aimrt::plugins::record_playback_plugin.