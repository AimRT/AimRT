// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdio>

#ifdef __linux__
  #include <unistd.h>

#else                   // windows
  #include <io.h>       // for _get_osfhandle, _fileno
  #include <windows.h>  // for FlushFileBuffers
#endif

namespace aimrt::runtime::core::logger {

inline bool Fsync(FILE *fp) {
#ifdef _WIN32
  return FlushFileBuffers(reinterpret_cast<HANDLE>(_get_osfhandle(_fileno(fp)))) != 0;
#else
  return ::fsync(fileno(fp)) == 0;
#endif
}

}  // namespace aimrt::runtime::core::logger