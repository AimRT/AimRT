# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get pybind11 ...")

set(pybind11_DOWNLOAD_URL
    "https://github.com/pybind/pybind11/archive/v2.13.1.tar.gz"
    CACHE STRING "")

if(pybind11_LOCAL_SOURCE)
  FetchContent_Declare(
    pybind11
    SOURCE_DIR ${pybind11_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    pybind11
    URL ${pybind11_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

# Wrap it in a function to restrict the scope of the variables
function(get_pybind11)
  FetchContent_GetProperties(pybind11)
  if(NOT pybind11_POPULATED)
    FetchContent_MakeAvailable(pybind11)
  endif()
endfunction()

get_pybind11()
