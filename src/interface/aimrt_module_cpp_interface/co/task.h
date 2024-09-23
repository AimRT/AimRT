// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#ifdef AIMRT_EXECUTOR_USE_STDEXEC

  #include <exec/task.hpp>

namespace aimrt::co {

template <typename T>
using Task = typename exec::task<T>;

}

#else

  #include <unifex/task.hpp>

namespace aimrt::co {

template <typename T>
using Task = typename unifex::task<T>;

}

#endif
