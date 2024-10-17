// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>
#include <unordered_map>

#include "aimrt_module_cpp_interface/channel/channel_context.h"
#include "aimrt_module_cpp_interface/util/type_support.h"

namespace aimrt::runtime::core::channel {

/**
 * @brief Topic base info
 *
 */
struct TopicInfo {
  std::string msg_type;
  std::string topic_name;
  std::string pkg_path;
  std::string module_name;

  aimrt::util::TypeSupportRef msg_type_support_ref;
};

/**
 * @brief Sub/Pub msg info with cache
 *
 */
struct MsgWrapper {
  /// Topic base info
  const TopicInfo& info;

  /// pointer to msg, may be nullptr
  const void* msg_ptr = nullptr;

  /// ref to ctx, may be empty
  aimrt::channel::ContextRef ctx_ref;

  /// serialization cache
  std::unordered_map<
      std::string,
      std::shared_ptr<aimrt::util::BufferArrayView>,
      aimrt::common::util::StringHash,
      std::equal_to<>>
      serialization_cache;

  /// msg cache
  std::shared_ptr<void> msg_cache_ptr;
};

}  // namespace aimrt::runtime::core::channel
