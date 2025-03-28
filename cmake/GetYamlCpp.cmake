# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get yaml-cpp ...")

set(yaml-cpp_DOWNLOAD_URL
    "https://github.com/jbeder/yaml-cpp/archive/0.8.0.tar.gz"
    CACHE STRING "")

if(yaml-cpp_LOCAL_SOURCE)
  FetchContent_Declare(
    yaml-cpp
    SOURCE_DIR ${yaml-cpp_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    yaml-cpp
    URL ${yaml-cpp_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

# Wrap it in a function to restrict the scope of the variables
function(get_yaml_cpp)
  FetchContent_GetProperties(yaml-cpp)
  if(NOT yaml-cpp_POPULATED)
    set(BUILD_TESTING OFF)

    set(YAML_CPP_BUILD_TESTS OFF)

    set(YAML_CPP_BUILD_TOOLS OFF)

    set(YAML_CPP_INSTALL ${AIMRT_INSTALL})

    set(YAML_CPP_FORMAT_SOURCE OFF)

    set(YAML_CPP_BUILD_CONTRIB OFF)

    FetchContent_MakeAvailable(yaml-cpp)
  endif()
endfunction()

get_yaml_cpp()

# import targets:
# yaml-cpp::yaml-cpp
