cmake_minimum_required(VERSION 3.22)

project(aimrt_iceoryx2_plugin)

# ============================================================================
# Iceoryx2 dependency - automatically fetched and built
# ============================================================================

# Include the auto-fetch module
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/FetchIceoryx2.cmake)

# ============================================================================
# Plugin shared library
# ============================================================================

add_library(${PROJECT_NAME} SHARED
    iceoryx2_plugin_main.cc
    iceoryx2_plugin.cc
    iceoryx2_channel_backend.cc
    global.cc
)

# Add dependency on iceoryx2 cargo build
add_dependencies(${PROJECT_NAME} iceoryx2_cargo_build)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..      # For iceoryx2_plugin/xxx.h includes
    ${CMAKE_CURRENT_SOURCE_DIR}/../..   # For src/ includes (runtime/core, etc.)
    ${ICEORYX2_INCLUDE_DIRS}            # Iceoryx2 C++ headers
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    aimrt::runtime::core
    yaml-cpp
    ${ICEORYX2_LIBRARIES}
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
  
  add_dependencies(aimrt_iceoryx2_plugin_test iceoryx2_cargo_build)
  
  target_link_libraries(aimrt_iceoryx2_plugin_test
    PRIVATE
    ${PROJECT_NAME}
    GTest::gtest_main
    aimrt::runtime::core
    ${ICEORYX2_LIBRARIES}
    yaml-cpp
  )
  
  target_include_directories(aimrt_iceoryx2_plugin_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${ICEORYX2_INCLUDE_DIRS}
  )
  
  add_test(NAME aimrt_iceoryx2_plugin_test COMMAND aimrt_iceoryx2_plugin_test)
endif()
