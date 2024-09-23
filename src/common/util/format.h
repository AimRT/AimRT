// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#ifdef AIMRT_USE_FMT_LIB
  #include "fmt/chrono.h"
  #include "fmt/core.h"
  #include "fmt/format.h"

  #define aimrt_fmt fmt

#else
  #include <format>

  #define aimrt_fmt std
#endif
