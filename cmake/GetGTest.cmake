# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get googletest ...")

set(googletest_DOWNLOAD_URL
    "https://github.com/google/googletest/archive/v1.13.0.tar.gz"
    CACHE STRING "")

if(googletest_LOCAL_SOURCE)
  FetchContent_Declare(
    googletest
    SOURCE_DIR ${googletest_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    googletest
    URL ${googletest_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

# Wrap it in a function to restrict the scope of the variables
function(get_googletest)
  FetchContent_GetProperties(googletest)
  if(NOT googletest_POPULATED)
    if(WIN32)
      set(gtest_force_shared_crt ON)
    endif()
    set(INSTALL_GTEST OFF)

    FetchContent_MakeAvailable(googletest)
  endif()
endfunction()

get_googletest()

# import targets:
# GTest::gtest
# GTest::gtest_main
# GTest::gmock
# GTest::gmock_main

# add xxxlib_test target for xxxlib
function(add_gtest_target)
  cmake_parse_arguments(ARG "" "TEST_TARGET" "TEST_SRC;INC_DIR" ${ARGN})
  set(TEST_TARGET_NAME ${ARG_TEST_TARGET}_test)

  get_target_property(TEST_TARGET_TYPE ${ARG_TEST_TARGET} TYPE)
  if(${TEST_TARGET_TYPE} STREQUAL SHARED_LIBRARY)
    get_target_property(TEST_TARGET_SOURCES ${ARG_TEST_TARGET} SOURCES)
    add_executable(${TEST_TARGET_NAME} ${ARG_TEST_SRC} ${TEST_TARGET_SOURCES})

    get_target_property(TEST_TARGET_INCLUDE_DIRECTORIES ${ARG_TEST_TARGET} INCLUDE_DIRECTORIES)
    target_include_directories(${TEST_TARGET_NAME} PRIVATE ${ARG_INC_DIR} ${TEST_TARGET_INCLUDE_DIRECTORIES})

    get_target_property(TEST_TARGET_LINK_LIBRARIES ${ARG_TEST_TARGET} LINK_LIBRARIES)
    target_link_libraries(${TEST_TARGET_NAME} PRIVATE ${TEST_TARGET_LINK_LIBRARIES} GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main)
  else()
    add_executable(${TEST_TARGET_NAME} ${ARG_TEST_SRC})
    target_include_directories(${TEST_TARGET_NAME} PRIVATE ${ARG_INC_DIR})
    target_link_libraries(${TEST_TARGET_NAME} PRIVATE ${ARG_TEST_TARGET} GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main)
  endif()

  add_test(NAME ${TEST_TARGET_NAME} COMMAND $<TARGET_FILE:${TEST_TARGET_NAME}>)
endfunction()
