# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "Getting irobot_events_executor...")

set(events_executor_GIT_URL
    "https://github.com/irobot-ros/events-executor/archive/b2ef94e1ecee3aa3369c680343df46035998ddc0.tar.gz"
    CACHE STRING "")

if(events_executor_LOCAL_SOURCE)
  FetchContent_Declare(
    irobot_events_executor
    SOURCE_DIR ${events_executor_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    irobot_events_executor
    URL ${events_executor_GIT_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

function(get_events_executor)
  FetchContent_GetProperties(irobot_events_executor)
  if(NOT irobot_events_executor_POPULATED)
    FetchContent_Populate(irobot_events_executor)

    set(_events_exec_cpp "${irobot_events_executor_SOURCE_DIR}/irobot_events_executor/src/rclcpp/executors/events_executor/events_executor.cpp")
    if(EXISTS "${_events_exec_cpp}")
      file(READ "${_events_exec_cpp}" CONTENTS)
      string(REPLACE "executor_notifier_->add_guard_condition(interrupt_guard_condition_.get());" "executor_notifier_->add_guard_condition(&interrupt_guard_condition_);" CONTENTS
                     "${CONTENTS}")
      file(WRITE "${_events_exec_cpp}" "${CONTENTS}")
    endif()

    set(BUILD_TESTING
        OFF
        CACHE BOOL "Disable testing for deps" FORCE)
    set(AMENT_CMAKE_COPY_DEPENDENCIES
        OFF
        CACHE BOOL "" FORCE)

    add_subdirectory("${irobot_events_executor_SOURCE_DIR}/irobot_events_executor" "${irobot_events_executor_BINARY_DIR}_events_exec")

    set(_mock_config_dir "${CMAKE_CURRENT_BINARY_DIR}/irobot_events_executor_mock")
    file(MAKE_DIRECTORY "${_mock_config_dir}")

    file(
      WRITE "${_mock_config_dir}/irobot_events_executorConfig.cmake"
      "
      set(irobot_events_executor_FOUND TRUE)

      if(TARGET irobot_events_executor)
        if(NOT TARGET irobot_events_executor::irobot_events_executor)
          add_library(irobot_events_executor::irobot_events_executor ALIAS irobot_events_executor)
        endif()
      endif()
    ")

    set(irobot_events_executor_DIR
        "${_mock_config_dir}"
        CACHE PATH "Hack for finding irobot_events_executor built in-tree" FORCE)

    add_subdirectory("${irobot_events_executor_SOURCE_DIR}/irobot_lock_free_events_queue" "${irobot_events_executor_BINARY_DIR}_lock_free_queue")

  endif()
endfunction()

# 执行函数
get_events_executor()
