// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#ifdef AIMRT_EXECUTOR_USE_STDEXEC

  #include <stdexec/execution.hpp>

namespace aimrt::co {

inline constexpr auto& SyncWait = stdexec::sync_wait;

}

#else

  #include <unifex/sync_wait.hpp>

namespace aimrt::co {

inline constexpr auto& SyncWait = unifex::sync_wait;

}  // namespace aimrt::co

#endif
