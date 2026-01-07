# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "Getting irobot_events_executor...")

set(events_executor_GIT_URL
    "https://github.com/irobot-ros/events-executor.git"
    CACHE STRING "")

set(events_executor_GIT_TAG
    "main"
    CACHE STRING "")

if(events_executor_LOCAL_SOURCE)
  FetchContent_Declare(
    irobot_events_executor
    SOURCE_DIR ${events_executor_LOCAL_SOURCE} SOURCE_SUBDIR irobot_events_executor
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    irobot_events_executor
    GIT_REPOSITORY
    ${events_executor_GIT_URL}
    GIT_TAG
    ${events_executor_GIT_TAG}
    SOURCE_SUBDIR
    irobot_events_executor
    OVERRIDE_FIND_PACKAGE)
endif()

function(get_events_executor)
  FetchContent_GetProperties(irobot_events_executor)
  if(NOT irobot_events_executor_POPULATED)
    # Populate first so irobot_events_executor_SOURCE_DIR is valid.
    FetchContent_Populate(irobot_events_executor)

    set(_events_exec_cpp
        "${irobot_events_executor_SOURCE_DIR}/irobot_events_executor/src/rclcpp/executors/events_executor/events_executor.cpp")
    if(EXISTS "${_events_exec_cpp}")
      file(READ "${_events_exec_cpp}" CONTENTS)
      string(REPLACE
             "executor_notifier_->add_guard_condition(interrupt_guard_condition_.get());"
             "executor_notifier_->add_guard_condition(&interrupt_guard_condition_);"
             CONTENTS "${CONTENTS}")
      file(WRITE "${_events_exec_cpp}" "${CONTENTS}")
    endif()

    set(BUILD_TESTING
        OFF
        CACHE BOOL "Disable testing for deps" FORCE)
    set(AMENT_CMAKE_COPY_DEPENDENCIES
        OFF
        CACHE BOOL "" FORCE)

    # Equivalent to FetchContent_MakeAvailable(), but avoids repopulating.
    add_subdirectory("${irobot_events_executor_SOURCE_DIR}/irobot_events_executor"
                     "${irobot_events_executor_BINARY_DIR}")
  endif()
endfunction()

# 执行函数
get_events_executor()
