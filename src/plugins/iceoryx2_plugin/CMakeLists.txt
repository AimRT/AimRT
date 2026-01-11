cmake_minimum_required(VERSION 3.22)

project(aimrt_iceoryx2_plugin)

# Iceoryx2 is located in AimRT/src/third_party/iceoryx2
# Path from plugins/iceoryx2_plugin -> ../../third_party/iceoryx2
set(ICEORYX2_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/iceoryx2)

if(NOT EXISTS "${ICEORYX2_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Iceoryx2 source not found at ${ICEORYX2_SOURCE_DIR}")
endif()

# Build iceoryx2 with C++ bindings
set(BUILD_CXX ON CACHE BOOL "Build iceoryx2 C++ bindings")
add_subdirectory(${ICEORYX2_SOURCE_DIR} iceoryx2_build EXCLUDE_FROM_ALL)

# Plugin shared library
add_library(${PROJECT_NAME} SHARED
    iceoryx2_plugin_main.cc
    iceoryx2_plugin.cc
    iceoryx2_channel_backend.cc
    global.cc
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..   # For iceoryx2_plugin/xxx.h includes
    ${CMAKE_CURRENT_SOURCE_DIR}/../..  # For src/ includes (runtime/core, etc.)
    ${ICEORYX2_SOURCE_DIR}/iceoryx2-cxx/include
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
    ${ICEORYX2_SOURCE_DIR}/iceoryx2-cxx/include
  )
  add_test(NAME aimrt_iceoryx2_plugin_test COMMAND aimrt_iceoryx2_plugin_test)
endif()
