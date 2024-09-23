# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get gflags ...")

set(gflags_DOWNLOAD_URL
    "https://github.com/gflags/gflags/archive/v2.2.2.tar.gz"
    CACHE STRING "")

if(gflags_LOCAL_SOURCE)
  FetchContent_Declare(
    gflags
    SOURCE_DIR ${gflags_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    gflags
    URL ${gflags_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

FetchContent_GetProperties(gflags)
if(NOT gflags_POPULATED)
  FetchContent_Populate(gflags)

  set(BUILD_TESTING
      OFF
      CACHE BOOL "")

  file(READ ${gflags_SOURCE_DIR}/CMakeLists.txt TMP_VAR)
  string(REPLACE "  set (PKGCONFIG_INSTALL_DIR " "# set (PKGCONFIG_INSTALL_DIR " TMP_VAR "${TMP_VAR}")
  file(WRITE ${gflags_SOURCE_DIR}/CMakeLists.txt "${TMP_VAR}")

  add_subdirectory(${gflags_SOURCE_DIR} ${gflags_BINARY_DIR})

endif()

# import targets:
# gflags::gflags
