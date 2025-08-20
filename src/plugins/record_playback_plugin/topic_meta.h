// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "aimrt_module_cpp_interface/util/buffer.h"

namespace aimrt::plugins::record_playback_plugin {

struct TopicMeta {
  uint64_t id = 0;
  std::string topic_name;
  std::string msg_type;
  std::string serialization_type;
  double sample_freq = 0;
};

}  // namespace aimrt::plugins::record_playback_plugin