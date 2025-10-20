// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once
#include "aimrt_module_cpp_interface/context/context.h"
#include "context/details/concepts.h"
#include "context/res/channel.h"
#include "unifex/sync_wait.hpp"

namespace aimrt::context::init {

template <concepts::DirectlySupportedType T>
context::res::Publisher<T> Publisher(const std::string_view& topic_name, std::source_location loc = std::source_location::current()) {
  return context::details::ExpectContext(loc)->pub(loc).Init<T>(topic_name);
}

template <concepts::DirectlySupportedType T>
context::res::Subscriber<T> Subscriber(const std::string_view& topic_name, std::source_location loc = std::source_location::current()) {
  return context::details::ExpectContext(loc)->sub(loc).Init<T>(topic_name);
}

}  // namespace aimrt::context::init