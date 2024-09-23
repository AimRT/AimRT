// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#ifdef AIMRT_EXECUTOR_USE_STDEXEC

  #include <exec/inline_scheduler.hpp>

namespace aimrt::co {

using InlineScheduler = exec::inline_scheduler;

}

#else

  #include <unifex/inline_scheduler.hpp>

namespace aimrt::co {

using InlineScheduler = unifex::inline_scheduler;

}  // namespace aimrt::co

#endif
