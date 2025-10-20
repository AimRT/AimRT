// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once
#include <source_location>
#include "aimrt_module_cpp_interface/context/details/thread_context.h"

namespace aimrt::context {

bool Ok(std::source_location loc = std::source_location::current()) {
  return details::ExpectContext(loc)->Ok();
}

}  // namespace aimrt::context