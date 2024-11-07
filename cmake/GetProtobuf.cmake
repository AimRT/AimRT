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

# Wrap it in a function to restrict the scope of the variables
function(get_protobuf)
  FetchContent_GetProperties(protobuf)
  if(NOT protobuf_POPULATED)
    set(protobuf_BUILD_TESTS OFF)

    set(protobuf_BUILD_CONFORMANCE OFF)

    set(protobuf_DISABLE_RTTI OFF)

    set(protobuf_WITH_ZLIB OFF)

    set(protobuf_MSVC_STATIC_RUNTIME OFF)

    set(protobuf_INSTALL ${AIMRT_INSTALL})

    set(protobuf_VERBOSE ON)

    FetchContent_MakeAvailable(protobuf)
  endif()
endfunction()

get_protobuf()

# import targets:
# protobuf::libprotobuf
# protobuf::libprotobuf-lite
# protobuf::libprotoc
# protobuf::protoc
