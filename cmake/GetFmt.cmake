# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get fmt ...")

set(fmt_DOWNLOAD_URL
    "https://github.com/fmtlib/fmt/archive/10.2.1.tar.gz"
    CACHE STRING "")

if(fmt_LOCAL_SOURCE)
  FetchContent_Declare(
    fmt
    SOURCE_DIR ${fmt_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    fmt
    URL ${fmt_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

# Wrap it in a function to restrict the scope of the variables
function(get_fmt)
  FetchContent_GetProperties(fmt)
  if(NOT fmt_POPULATED)
    set(FMT_MASTER_PROJECT OFF)

    set(FMT_INSTALL ON)

    FetchContent_MakeAvailable(fmt)
  endif()
endfunction()

get_fmt()

# import targets：
# fmt::fmt
