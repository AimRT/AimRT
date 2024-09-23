# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get jsoncpp ...")

set(jsoncpp_DOWNLOAD_URL
    "https://github.com/open-source-parsers/jsoncpp/archive/1.9.5.tar.gz"
    CACHE STRING "")

if(jsoncpp_LOCAL_SOURCE)
  FetchContent_Declare(
    jsoncpp
    SOURCE_DIR ${jsoncpp_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    jsoncpp
    URL ${jsoncpp_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

FetchContent_GetProperties(jsoncpp)
if(NOT jsoncpp_POPULATED)
  set(JSONCPP_WITH_TESTS
      OFF
      CACHE BOOL "")
  set(JSONCPP_WITH_POST_BUILD_UNITTEST
      OFF
      CACHE BOOL "")

  set(BUILD_OBJECT_LIBS
      OFF
      CACHE BOOL "")

  if(BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS
        ON
        CACHE BOOL "")
    set(BUILD_STATIC_LIBS
        OFF
        CACHE BOOL "")
  else()
    set(BUILD_SHARED_LIBS
        OFF
        CACHE BOOL "")
    set(BUILD_STATIC_LIBS
        ON
        CACHE BOOL "")
  endif()

  FetchContent_MakeAvailable(jsoncpp)

  if(TARGET jsoncpp_static)
    add_library(jsoncpp::jsoncpp ALIAS jsoncpp_static)
  elseif(TARGET jsoncpp_lib)
    add_library(jsoncpp::jsoncpp ALIAS jsoncpp_lib)
  endif()
endif()

# import targets:
# jsoncpp::jsoncpp
