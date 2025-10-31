// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <source_location>

namespace aimrt::context {

class Context;

class OpBase {
 protected:
  OpBase(Context& ctx, std::source_location loc) noexcept
      : ctx_(ctx), loc_(loc) {}

  Context& ctx_;
  std::source_location loc_;
};

}  // namespace aimrt::context
