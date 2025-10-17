// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/channel/channel_context.h"
#include "aimrt_module_cpp_interface/context/res/details/base.h"

namespace aimrt::context::res {

template <class T>
class Channel : public details::Base {
 public:
  using MessageType = T;
  using details::Base::Base;

  // Publish helpers to enable publisher_.publish(msg)
  void Publish(const T& msg) const;
  void Publish(aimrt::channel::ContextRef ch_ctx, const T& msg) const;
};

}  // namespace aimrt::context::res
