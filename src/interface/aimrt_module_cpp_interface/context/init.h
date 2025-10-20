// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once
#include <source_location>
#include <string>
#include <string_view>
#include "aimrt_module_cpp_interface/context/details/concepts.h"
#include "aimrt_module_cpp_interface/context/details/thread_context.h"
#include "aimrt_module_cpp_interface/context/res/channel.h"

namespace aimrt::context::init {

// template <concepts::DirectlySupportedType T>
// [[nodiscard]] context::res::Publisher<T> Publisher(const std::string_view& topic_name, std::source_location loc = std::source_location::current())
// {
//     return {aimrt::context::details::ExpectContext(loc)->pub(loc).Init<T>(topic_name)};
// }

}  // namespace aimrt::context::init