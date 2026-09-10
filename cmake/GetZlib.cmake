# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include_guard(GLOBAL)

include(FetchContent)

message(STATUS "get zlib ...")

set(ZLIB_VERSION
    "1.3.1"
    CACHE STRING "zlib version to use")
set(zlib_DOWNLOAD_URL
    "https://github.com/madler/zlib/releases/download/v${ZLIB_VERSION}/zlib-${ZLIB_VERSION}.tar.gz"
    CACHE STRING "")

if(zlib_LOCAL_SOURCE)
  FetchContent_Declare(
    zlib
    SOURCE_DIR ${zlib_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    zlib
    URL ${zlib_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

# Wrap it in a function to restrict the scope of the variables
function(get_zlib)
  FetchContent_GetProperties(zlib)
  if(NOT zlib_POPULATED)
    FetchContent_Populate(zlib)

    set(ZLIB_BUILD_EXAMPLES OFF)

    # zlib is only linked into aimrt targets, its own install rules are not needed
    set(SKIP_INSTALL_ALL ON)

    add_subdirectory(${zlib_SOURCE_DIR} ${zlib_BINARY_DIR} EXCLUDE_FROM_ALL)

    # wrap the static lib in an imported target, so that it is referenced by name in
    # aimrt's export set instead of being required to be a part of it
    if(NOT TARGET ZLIB::ZLIB)
      add_library(ZLIB::ZLIB INTERFACE IMPORTED GLOBAL)
      target_link_libraries(ZLIB::ZLIB INTERFACE zlibstatic)
    endif()
  endif()
endfunction()

get_zlib()

# import targets:
# ZLIB::ZLIB
