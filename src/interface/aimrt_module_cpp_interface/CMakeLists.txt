# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Get namespace
get_namespace(CUR_SUPERIOR_NAMESPACE)
string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})

# Set target name
set(CUR_TARGET_NAME ${CUR_SUPERIOR_NAMESPACE_UNDERLINE}_${CUR_DIR})
set(CUR_TARGET_ALIAS_NAME ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR})

# Set file collection
file(GLOB_RECURSE head_files ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE test_files ${CMAKE_CURRENT_SOURCE_DIR}/*_test.cc)

# Add target
add_library(${CUR_TARGET_NAME} INTERFACE)
add_library(${CUR_TARGET_ALIAS_NAME} ALIAS ${CUR_TARGET_NAME})

# Set version of target

if(AIMRT_CONTEXT_DEFINE_GLOBAL_LOGGER_HANDLE)
  target_compile_definitions(${CUR_TARGET_NAME} INTERFACE AIMRT_RUNTIME_VERSION="${AIMRT_RUNTIME_VERSION}" AIMRT_CONTEXT_DEFINE_GLOBAL_LOGGER_HANDLE)
else()
  target_compile_definitions(${CUR_TARGET_NAME} INTERFACE AIMRT_RUNTIME_VERSION="${AIMRT_RUNTIME_VERSION}")
endif()

# Set include path of target
target_include_directories(
  ${CUR_TARGET_NAME}
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
            $<INSTALL_INTERFACE:include/aimrt>)

# Set head files of target
target_sources(${CUR_TARGET_NAME} INTERFACE FILE_SET HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} FILES ${head_files})

# Set link libraries of target
target_link_libraries(
  ${CUR_TARGET_NAME}
  INTERFACE aimrt::common::util
            aimrt::interface::aimrt_module_c_interface)

# Executor
if(AIMRT_EXECUTOR_USE_STDEXEC)
  target_link_libraries(${CUR_TARGET_NAME} INTERFACE STDEXEC::stdexec)
  target_compile_definitions(${CUR_TARGET_NAME} INTERFACE AIMRT_EXECUTOR_USE_STDEXEC)
else()
  target_link_libraries(${CUR_TARGET_NAME} INTERFACE unifex::unifex)
endif()

# Set installation of target
if(AIMRT_INSTALL)
  set_property(TARGET ${CUR_TARGET_NAME} PROPERTY EXPORT_NAME ${CUR_TARGET_ALIAS_NAME})
  install(
    TARGETS ${CUR_TARGET_NAME}
    EXPORT ${INSTALL_CONFIG_NAME}
    FILE_SET HEADERS
    DESTINATION include/aimrt/${CUR_DIR})
endif()

# Set test of target
if(AIMRT_BUILD_TESTS AND test_files)
  add_gtest_target(TEST_TARGET ${CUR_TARGET_NAME} TEST_SRC ${test_files})
endif()
