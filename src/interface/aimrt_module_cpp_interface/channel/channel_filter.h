// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <functional>

#include "aimrt_module_cpp_interface/channel/channel_context.h"

namespace aimrt::channel {

using ChannelHandle = std::function<void(ContextRef, const void*)>;
using ChannelFilter = std::function<void(ContextRef, const void*, const ChannelHandle&)>;

class ChannelFilterManager {
 public:
  ChannelFilterManager()
      : final_filter_([](ContextRef ctx_ref, const void* msg_ptr, const ChannelHandle& h) -> void {
          h(ctx_ref, msg_ptr);
        }) {}
  ~ChannelFilterManager() = default;
  ChannelFilterManager(const ChannelFilterManager&) = delete;
  ChannelFilterManager& operator=(const ChannelFilterManager&) = delete;

  void RegisterFilter(ChannelFilter&& filter) {
    final_filter_ = [final_filter{std::move(final_filter_)}, cur_filter{std::move(filter)}](
                        ContextRef ctx_ref, const void* msg_ptr, const ChannelHandle& h) -> void {
      cur_filter(ctx_ref, msg_ptr, [&final_filter, &h](ContextRef ctx_ref, const void* msg_ptr) -> void {
        final_filter(ctx_ref, msg_ptr, h);
      });
    };
  }

  void InvokeChannel(const ChannelHandle& h, ContextRef ctx_ref, const void* msg_ptr) const {
    final_filter_(ctx_ref, msg_ptr, h);
  }

  void Clear() {
    final_filter_ = ChannelFilter();
  }

 private:
  ChannelFilter final_filter_;
};

}  // namespace aimrt::channel