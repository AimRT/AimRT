# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get TBB ...")

set(tbb_DOWNLOAD_URL
    "https://github.com/oneapi-src/oneTBB/archive/v2021.13.0.tar.gz"
    CACHE STRING "")

if(tbb_LOCAL_SOURCE)
  FetchContent_Declare(
    tbb
    SOURCE_DIR ${tbb_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    tbb
    URL ${tbb_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

FetchContent_GetProperties(tbb)
if(NOT tbb_POPULATED)
  set(TBB_TEST
      OFF
      CACHE BOOL "")

  set(TBB_DIR
      ""
      CACHE STRING "" FORCE)

  set(TBB_INSTALL
      OFF
      CACHE BOOL "")

  set(TBB_STRICT
      OFF
      CACHE BOOL "")

  FetchContent_MakeAvailable(tbb)
endif()

# import targets:
# TBB::tbb
