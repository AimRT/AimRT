# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get libunifex ...")

set(libunifex_DOWNLOAD_URL
    "https://github.com/facebookexperimental/libunifex/archive/591ec09e7d51858ad05be979d4034574215f5971.tar.gz"
    CACHE STRING "")

if(libunifex_LOCAL_SOURCE)
  FetchContent_Declare(
    libunifex
    SOURCE_DIR ${libunifex_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    libunifex
    URL ${libunifex_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

FetchContent_GetProperties(libunifex)
if(NOT libunifex_POPULATED)
  set(UNIFEX_BUILD_EXAMPLES
      OFF
      CACHE BOOL "")

  FetchContent_MakeAvailable(libunifex)

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(unifex PRIVATE -Wno-unused-but-set-variable)
  endif()

  add_library(unifex::unifex ALIAS unifex)
endif()

# import targets:
# unifex::unifex
