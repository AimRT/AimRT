# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get iceoryx...")

# fetch iceoryx
set(iceoryx_DOWNLOAD_URL
    "https://github.com/eclipse-iceoryx/iceoryx/archive/refs/tags/v2.0.6.tar.gz"
    CACHE STRING "")

set(DOWNLOAD_TOML_LIB OFF)

if(iceoryx_LOCAL_SOURCE)
  FetchContent_Declare(
    iceoryx
    SOURCE_DIR ${iceoryx_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    iceoryx
    URL ${iceoryx_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    OVERRIDE_FIND_PACKAGE)
endif()

FetchContent_GetProperties(iceoryx)
if(NOT iceoryx_POPULATED)

  FetchContent_Populate(iceoryx)

  # iceoryx‘s cmake file in ./iceoryx_meta
  add_subdirectory(${iceoryx_SOURCE_DIR}/iceoryx_meta ${iceoryx_BINARY_DIR})

endif()
