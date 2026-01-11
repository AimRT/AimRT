cmake_minimum_required(VERSION 3.22)

project(aimrt_iceoryx2_plugin)

# ============================================================================
# Iceoryx2 dependency - via FetchContent using iceoryx2's native CMake
# ============================================================================

include(FetchContent)

set(ICEORYX2_VERSION "v0.8.0" CACHE STRING "Iceoryx2 version to fetch")

# iceoryx2 CMake options
set(IOX2_CXX_STD_VERSION 17 CACHE STRING "C++ standard version for iceoryx2")
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static libraries")

# Check if already fetched by parent
if(NOT TARGET iceoryx2-cxx::static-lib-cxx)
  message(STATUS "Fetching iceoryx2 ${ICEORYX2_VERSION} from GitHub...")
  FetchContent_Declare(
    iceoryx2_src
    GIT_REPOSITORY https://github.com/eclipse-iceoryx/iceoryx2.git
    GIT_TAG        ${ICEORYX2_VERSION}
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
  )
  FetchContent_MakeAvailable(iceoryx2_src)
endif()

# ============================================================================
# Plugin shared library
# ============================================================================

add_library(${PROJECT_NAME} SHARED
    iceoryx2_plugin_main.cc
    iceoryx2_plugin.cc
    iceoryx2_channel_backend.cc
    global.cc
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..      # For iceoryx2_plugin/xxx.h includes
    ${CMAKE_CURRENT_SOURCE_DIR}/../..   # For src/ includes (runtime/core, etc.)
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    aimrt::runtime::core
    yaml-cpp
    iceoryx2-cxx::static-lib-cxx
)

set_target_properties(${PROJECT_NAME}
    PROPERTIES
    PREFIX ""
    OUTPUT_NAME "aimrt_iceoryx2_plugin"
)

# ============================================================================
# Unit tests
# ============================================================================

if(BUILD_TESTING)
  add_executable(aimrt_iceoryx2_plugin_test test/iceoryx2_plugin_test.cc)
  
  target_link_libraries(aimrt_iceoryx2_plugin_test
    PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
    aimrt::runtime::core
    iceoryx2-cxx::static-lib-cxx
    yaml-cpp
  )
  
  target_include_directories(aimrt_iceoryx2_plugin_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
  )
  
  add_test(NAME aimrt_iceoryx2_plugin_test COMMAND aimrt_iceoryx2_plugin_test)
endif()
