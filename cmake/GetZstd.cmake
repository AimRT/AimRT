# Copyright (c) 2024
# All rights reserved.

include_guard(GLOBAL)

include(FetchContent)

message(STATUS "get zstd ...")

set(ZSTD_VERSION
    "1.5.7"
    CACHE STRING "zstd version to use")
set(zstd_DOWNLOAD_URL
    "https://github.com/facebook/zstd/releases/download/v${ZSTD_VERSION}/zstd-${ZSTD_VERSION}.tar.gz"
    CACHE STRING "")

if(zstd_LOCAL_SOURCE)
  FetchContent_Declare(
    zstd
    SOURCE_DIR ${zstd_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    zstd
    URL ${zstd_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

function(get_zstd)
  FetchContent_GetProperties(zstd)
  if(NOT zstd_POPULATED)
    FetchContent_Populate(zstd)

    set(ZSTD_BUILD_PROGRAMS
        OFF
        CACHE BOOL "" FORCE)
    set(ZSTD_BUILD_SHARED
        OFF
        CACHE BOOL "" FORCE)
    set(ZSTD_BUILD_STATIC
        ON
        CACHE BOOL "" FORCE)
    set(ZSTD_BUILD_TESTS
        OFF
        CACHE BOOL "" FORCE)

    add_subdirectory(${zstd_SOURCE_DIR}/build/cmake ${zstd_BINARY_DIR}/build/cmake EXCLUDE_FROM_ALL)

    # wrap the static lib in an imported target, so that it is referenced by name in
    # aimrt's export set instead of being required to be a part of it
    if(NOT TARGET zstd::libzstd)
      add_library(zstd::libzstd INTERFACE IMPORTED GLOBAL)
      target_link_libraries(zstd::libzstd INTERFACE libzstd_static)
    endif()
  endif()
endfunction()

get_zstd()

# import targets:
# zstd::libzstd
