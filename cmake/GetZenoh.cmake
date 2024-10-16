# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "Getting zenohc...")

# fetch zenoh-c
set(zenohc_DOWNLOAD_URL
    "https://github.com/eclipse-zenoh/zenoh-c/archive/refs/tags/1.0.0.11.tar.gz"
    CACHE STRING "")

if(zenohc_LOCAL_SOURCE)
  FetchContent_Declare(
    zenohc
    SOURCE_DIR ${zenohc_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    zenohc
    URL ${zenohc_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    OVERRIDE_FIND_PACKAGE)
endif()

FetchContent_GetProperties(zenohc)
if(NOT zenohc_POPULATED)
  FetchContent_MakeAvailable(zenohc)
endif()
