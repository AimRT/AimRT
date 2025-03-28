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

# Wrap it in a function to restrict the scope of the variables
function(get_tbb)
  FetchContent_GetProperties(tbb)
  if(NOT tbb_POPULATED)
    set(TBB_TEST OFF)

    set(TBB_DIR "")

    set(TBB_INSTALL ${AIMRT_INSTALL})

    set(TBB_STRICT OFF)

    FetchContent_MakeAvailable(tbb)
  endif()
endfunction()

get_tbb()

# import targets:
# TBB::tbb
