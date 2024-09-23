# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get nghttp2 ...")

set(nghttp2_DOWNLOAD_URL
    "https://github.com/nghttp2/nghttp2/archive/refs/tags/v1.62.1.zip"
    CACHE STRING "")

if(nghttp2_LOCAL_SOURCE)
  FetchContent_Declare(
    nghttp2
    SOURCE_DIR ${nghttp2_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    nghttp2
    URL ${nghttp2_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

FetchContent_GetProperties(nghttp2)
if(NOT nghttp2_POPULATED)
  FetchContent_Populate(nghttp2)

  set(BUILD_SHARED_LIBS OFF)
  set(BUILD_STATIC_LIBS ON)

  # Avoid name conflict
  set(nghttp2_CMAKE_FILE "${nghttp2_SOURCE_DIR}/CMakeLists.txt")
  file(READ ${nghttp2_CMAKE_FILE} CONTENTS)
  string(REPLACE "add_custom_target(check COMMAND \${CMAKE_CTEST_COMMAND})" "" NEW_CONTENTS "${CONTENTS}")
  file(WRITE ${nghttp2_CMAKE_FILE} "${NEW_CONTENTS}")

  add_subdirectory(${nghttp2_SOURCE_DIR} ${nghttp2_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

# import targets:
# nghttp2::nghttp2
