# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get protobuf ...")

set(protobuf_DOWNLOAD_URL
    "https://github.com/protocolbuffers/protobuf/archive/v3.21.12.tar.gz"
    CACHE STRING "")

if(protobuf_LOCAL_SOURCE)
  FetchContent_Declare(
    protobuf
    SOURCE_DIR ${protobuf_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    protobuf
    URL ${protobuf_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

FetchContent_GetProperties(protobuf)
if(NOT protobuf_POPULATED)
  set(protobuf_BUILD_TESTS
      OFF
      CACHE BOOL "")
  set(protobuf_BUILD_CONFORMANCE
      OFF
      CACHE BOOL "")
  set(protobuf_BUILD_EXAMPLES
      OFF
      CACHE BOOL "")
  set(protobuf_DISABLE_RTTI
      OFF
      CACHE BOOL "")
  set(protobuf_WITH_ZLIB
      OFF
      CACHE BOOL "")
  set(protobuf_MSVC_STATIC_RUNTIME
      OFF
      CACHE BOOL "")
  set(protobuf_INSTALL
      ${AIMRT_INSTALL}
      CACHE BOOL "")
  set(protobuf_VERBOSE
      ON
      CACHE BOOL "")

  FetchContent_MakeAvailable(protobuf)
endif()

# import targets:
# protobuf::libprotobuf
# protobuf::libprotobuf-lite
# protobuf::libprotoc
# protobuf::protoc
