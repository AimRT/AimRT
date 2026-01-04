# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get backward ...")

set(backward_DOWNLOAD_URL
    "https://github.com/bombela/backward-cpp/archive/refs/tags/v1.6.tar.gz"
    CACHE STRING "")

if(backward_LOCAL_SOURCE)
  FetchContent_Declare(
    backward
    SOURCE_DIR ${backward_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    backward
    URL ${backward_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

# Wrap it in a function to restrict the scope of the variables
function(get_backward)
  FetchContent_GetProperties(backward)
  if(NOT backward_POPULATED)
    FetchContent_Populate(backward)

    # Include BackwardConfig.cmake to get definitions and libraries
    include(${backward_SOURCE_DIR}/BackwardConfig.cmake)

    if(NOT TARGET Backward::Backward)
      add_library(Backward::Backward INTERFACE IMPORTED GLOBAL)
    endif()
    set_target_properties(Backward::Backward PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BACKWARD_INCLUDE_DIRS}" INTERFACE_COMPILE_DEFINITIONS "${BACKWARD_DEFINITIONS}")
    if(BACKWARD_HAS_EXTERNAL_LIBRARIES)
      set_target_properties(Backward::Backward PROPERTIES INTERFACE_LINK_LIBRARIES "${BACKWARD_LIBRARIES}")
    endif()
  endif()
endfunction()

get_backward()
