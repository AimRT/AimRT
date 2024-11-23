// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once
#include <string>

namespace aimrt::runtime::core::util {

struct TopicMetaKey {
  std::string topic_name;
  std::string msg_type;

  bool operator==(const TopicMetaKey& rhs) const {
    return topic_name == rhs.topic_name && msg_type == rhs.msg_type;
  }

  struct Hash {
    std::size_t operator()(const TopicMetaKey& k) const {
      return (std::hash<std::string>()(k.topic_name)) ^
             (std::hash<std::string>()(k.msg_type));
    }
  };
};

}  // namespace aimrt::runtime::core::util
