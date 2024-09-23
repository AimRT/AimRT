// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define AIMRT_TYPE_SUPPORT_PKG_EXPORT __attribute__((dllexport))
  #else
    #define AIMRT_TYPE_SUPPORT_PKG_EXPORT __declspec(dllexport)
  #endif
#else
  #define AIMRT_TYPE_SUPPORT_PKG_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
}
#endif
