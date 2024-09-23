# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "Searching for Boost...")

set(BOOST_VERSION 1.82.0)
set(boost_DOWNLOAD_URL
    "https://github.com/boostorg/boost/releases/download/boost-${BOOST_VERSION}/boost-${BOOST_VERSION}.tar.xz"
    CACHE STRING "")

find_package(Boost ${BOOST_VERSION} QUIET)

if(Boost_FOUND)
  if(Boost_VERSION VERSION_EQUAL ${BOOST_VERSION})
    message(STATUS "Found compatible Boost version ${Boost_VERSION}")
  else()
    message(WARNING "Found newer Boost version ${Boost_VERSION}. Using local version, but this may cause issues.")
  endif()
else()
  message(STATUS "Boost ${BOOST_VERSION} or greater not found locally. Downloading Boost ${BOOST_VERSION}")

  if(boost_LOCAL_SOURCE)
    FetchContent_Declare(
      boost
      SOURCE_DIR ${boost_LOCAL_SOURCE}
      OVERRIDE_FIND_PACKAGE)
    message(STATUS "Using local Boost source")
  else()
    FetchContent_Declare(
      boost
      URL ${boost_DOWNLOAD_URL}
      DOWNLOAD_EXTRACT_TIMESTAMP ON
      OVERRIDE_FIND_PACKAGE)
  endif()

  FetchContent_GetProperties(boost)
  if(NOT boost_POPULATED)
    set(BOOST_INCLUDE_LIBRARIES asio beast)

    set(Boost_USE_STATIC_LIBS
        ON
        CACHE BOOL "")

    FetchContent_MakeAvailable(boost)
  endif()
endif()
