# Copyright (c) 2024
# All rights reserved.

include(FetchContent)

message(STATUS "get mcap ...")

set(MCAP_VERSION
    "1.4.1"
    CACHE STRING "MCAP version to use")
set(MCAP_TAG "releases/cpp/v${MCAP_VERSION}")

if(MCAP_LOCAL_SOURCE)
  FetchContent_Declare(
    mcap
    SOURCE_DIR ${MCAP_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    mcap
    URL https://github.com/foxglove/mcap/archive/refs/tags/${MCAP_TAG}.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

function(get_mcap)
  FetchContent_GetProperties(mcap)
  if(NOT mcap_POPULATED)
    FetchContent_Populate(mcap)

    add_library(mcap INTERFACE)
    add_library(mcap::mcap ALIAS mcap)

    target_include_directories(mcap INTERFACE $<BUILD_INTERFACE:${mcap_SOURCE_DIR}/cpp/mcap/include> $<INSTALL_INTERFACE:include>)

    find_package(lz4 QUIET)
    if(NOT lz4_FOUND)
      target_compile_definitions(mcap INTERFACE "MCAP_COMPRESSION_NO_LZ4")
    else()
      target_link_libraries(mcap INTERFACE LZ4::lz4)
    endif()

    find_package(zstd QUIET)
    if(NOT zstd_FOUND)
      target_compile_definitions(mcap INTERFACE "MCAP_COMPRESSION_NO_ZSTD")
    else()
      target_link_libraries(mcap INTERFACE zstd::libzstd)
    endif()
  endif()
endfunction()

get_mcap()

# import targets:
# mcap::mcap
