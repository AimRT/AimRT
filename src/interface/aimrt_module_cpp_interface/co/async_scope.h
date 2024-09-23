// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#ifdef AIMRT_EXECUTOR_USE_STDEXEC

  #include <exec/async_scope.hpp>

namespace aimrt::co {

using AsyncScope = exec::async_scope;

  #define complete on_empty

}  // namespace aimrt::co

#else

  #include <unifex/async_scope.hpp>

namespace aimrt::co {

using AsyncScope = unifex::async_scope;

}  // namespace aimrt::co

#endif
