// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdio>

#ifdef _WIN32
  #include <io.h>       // for _get_osfhandle, _fileno
  #include <windows.h>  // for FlushFileBuffers
#else                   // Linux
  #include <unistd.h>
#endif
namespace aimrt::runtime::core::logger {

inline bool Fsync(FILE *fp) {
#ifdef _WIN32
  return FlushFileBuffers(reinterpret_cast<HANDLE>(_get_osfhandle(_fileno(fp)))) != 0;
#else
  return ::fsync(fileno(fp)) == 0;
#endif
}

// 文件夹分隔符定义
#if !defined(FOLDER_SEPS)
  #ifdef _WIN32
    #define FOLDER_SEPS "\\/"
  #else
    #define FOLDER_SEPS "/"
  #endif
#endif

inline constexpr const char folder_seps[] = FOLDER_SEPS;

}  // namespace aimrt::runtime::core::logger