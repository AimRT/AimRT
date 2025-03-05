# Copyright (c) 2024
# All rights reserved.

include(FetchContent)

message(STATUS "get mcap ...")

set(MCAP_VERSION
    "1.4.1"
    CACHE STRING "MCAP version to use")
set(MCAP_TAG "releases/cpp/v${MCAP_VERSION}")
set(mcap_DOWNLOAD_URL
    "https://github.com/foxglove/mcap/archive/refs/tags/${MCAP_TAG}.tar.gz"
    CACHE STRING "")

if(mcap_LOCAL_SOURCE)
  FetchContent_Declare(
    mcap
    SOURCE_DIR ${mcap_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    mcap
    URL ${mcap_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

set(ZSTD_VERSION
    "1.5.7"
    CACHE STRING "zstd version to use")
set(zstd_DOWNLOAD_URL
    "https://github.com/facebook/zstd/releases/download/v${ZSTD_VERSION}/zstd-${ZSTD_VERSION}.tar.gz"
    CACHE STRING "")

if(zstd_LOCAL_SOURCE)
  FetchContent_Declare(
    zstd
    SOURCE_DIR ${ZSTD_LOCAL_SOURCE}
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
    add_library(zstd::libzstd ALIAS libzstd_static)
  endif()
endfunction()

set(LZ4_VERSION
    "1.10.0"
    CACHE STRING "lz4 version to use")
set(lz4_DOWNLOAD_URL
    "https://github.com/lz4/lz4/releases/download/v${LZ4_VERSION}/lz4-${LZ4_VERSION}.tar.gz"
    CACHE STRING "")

if(lz4_LOCAL_SOURCE)
  FetchContent_Declare(
    lz4
    SOURCE_DIR ${lz4_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    lz4
    URL ${lz4_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    OVERRIDE_FIND_PACKAGE)
endif()

function(get_lz4)
  FetchContent_GetProperties(lz4)
  if(NOT lz4_POPULATED)
    FetchContent_Populate(lz4)

    set(LZ4_BUILD_CLI
        OFF
        CACHE BOOL "" FORCE)
    set(LZ4_BUILD_LEGACY_LZ4C
        OFF
        CACHE BOOL "" FORCE)
    set(BUILD_SHARED_LIBS
        OFF
        CACHE BOOL "" FORCE)
    set(LZ4_POSITION_INDEPENDENT_LIB
        ON
        CACHE BOOL "" FORCE)

    add_subdirectory(${lz4_SOURCE_DIR}/build/cmake ${lz4_BINARY_DIR} EXCLUDE_FROM_ALL)

    add_library(LZ4::lz4 ALIAS lz4_static)
  endif()
endfunction()

function(get_mcap)
  FetchContent_GetProperties(mcap)
  if(NOT mcap_POPULATED)
    FetchContent_Populate(mcap)

    add_library(mcap INTERFACE)
    add_library(mcap::mcap ALIAS mcap)

    target_include_directories(mcap INTERFACE $<BUILD_INTERFACE:${mcap_SOURCE_DIR}/cpp/mcap/include> $<INSTALL_INTERFACE:include>)

    get_lz4()
    get_zstd()

    target_link_libraries(mcap INTERFACE LZ4::lz4 zstd::libzstd)

  endif()
endfunction()

get_mcap()

# import targets:
# mcap::mcap
