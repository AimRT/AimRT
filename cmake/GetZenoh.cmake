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

# Wrap it in a function to restrict the scope of the variables
function(get_zenohc)
  FetchContent_GetProperties(zenohc)
  if(NOT zenohc_POPULATED)
    set(ZENOHC_BUILD_WITH_UNSTABLE_API
        TRUE
        CACHE BOOL "Enable unstable API")
    set(ZENOHC_BUILD_WITH_SHARED_MEMORY
        TRUE
        CACHE BOOL "Enable shared memory")
    FetchContent_MakeAvailable(zenohc)
  endif()
endfunction()

get_zenohc()
