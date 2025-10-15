// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <any>

#include "aimrt_module_cpp_interface/channel/channel_context.h"
#include "aimrt_module_cpp_interface/channel/channel_handle.h"

namespace aimrt::context {

struct ChannelContext {
  aimrt::channel::PublisherRef pub;
  aimrt::channel::SubscriberRef sub;
  std::any pub_f;
  std::any sub_f;
};

}  // namespace aimrt::context
