// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#ifdef AIMRT_EXECUTOR_USE_STDEXEC

  #include <stdexec/execution.hpp>

namespace aimrt::co {

inline constexpr auto& Then = stdexec::then;

}

#else

  #include <unifex/then.hpp>

namespace aimrt::co {

inline constexpr auto& Then = unifex::then;

}

#endif
