# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "Getting irobot_events_executor...")

# Check rclcpp major version to determine what to build
if(NOT DEFINED rclcpp_VERSION_MAJOR)
  message(FATAL_ERROR "rclcpp_VERSION_MAJOR is not defined. Please find_package(rclcpp) first.")
endif()

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

    # In Humble (rclcpp 16.x), build both events_executor and lock_free_events_queue
    # In Jazzy (rclcpp 28.x), only build lock_free_events_queue, use system events_executor
    if(rclcpp_VERSION_MAJOR EQUAL 16)
      message(STATUS "Building irobot_events_executor for Humble (rclcpp 16.x)")

      set(_events_exec_cpp "${irobot_events_executor_SOURCE_DIR}/irobot_events_executor/src/rclcpp/executors/events_executor/events_executor.cpp")
      if(EXISTS "${_events_exec_cpp}")
        file(READ "${_events_exec_cpp}" CONTENTS)
        string(REPLACE "executor_notifier_->add_guard_condition(interrupt_guard_condition_.get());" "executor_notifier_->add_guard_condition(&interrupt_guard_condition_);"
                       CONTENTS "${CONTENTS}")
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

      if(TARGET irobot_events_executor)
        if(NOT TARGET irobot_events_executor::irobot_events_executor)
          add_library(irobot_events_executor::irobot_events_executor ALIAS irobot_events_executor)
        endif()
      endif()

      set(irobot_events_executor_DIR
          "${_mock_config_dir}"
          CACHE PATH "Hack for finding irobot_events_executor built in-tree" FORCE)
    elseif(rclcpp_VERSION_MAJOR EQUAL 28)
      message(STATUS "Using header-only irobot_lock_free_events_queue for Jazzy (rclcpp 28.x), using system events_executor")

      # Patch lock_free_events_queue.hpp to use correct include path and namespace for Jazzy
      # In Jazzy, events_executor is in rclcpp/experimental/executors/events_executor/
      set(_lock_free_hpp "${irobot_events_executor_SOURCE_DIR}/irobot_lock_free_events_queue/include/irobot_lock_free_events_queue/lock_free_events_queue.hpp")
      if(EXISTS "${_lock_free_hpp}")
        file(READ "${_lock_free_hpp}" CONTENTS)
        # Replace include path
        string(REPLACE "#include \"rclcpp/executors/events_executor/events_queue.hpp\"" "#include \"rclcpp/experimental/executors/events_executor/events_queue.hpp\"" CONTENTS
                       "${CONTENTS}")
        # Replace namespace paths: rclcpp::executors:: -> rclcpp::experimental::executors::
        string(REPLACE "rclcpp::executors::" "rclcpp::experimental::executors::" CONTENTS "${CONTENTS}")
        file(WRITE "${_lock_free_hpp}" "${CONTENTS}")
        message(STATUS "Patched lock_free_events_queue.hpp for Jazzy compatibility (include path and namespace)")
      endif()

      # irobot_lock_free_events_queue is header-only, just create an interface target with include directory
      if(NOT TARGET irobot_events_executor::irobot_lock_free_events_queue)
        add_library(irobot_events_executor::irobot_lock_free_events_queue INTERFACE IMPORTED GLOBAL)
        target_include_directories(irobot_events_executor::irobot_lock_free_events_queue INTERFACE "${irobot_events_executor_SOURCE_DIR}/irobot_lock_free_events_queue/include")
      endif()
    else()
      message(FATAL_ERROR "Unsupported rclcpp major version: ${rclcpp_VERSION_MAJOR} in GetEventsExecutor. Only rclcpp 16.x (Humble) and 28.x (Jazzy) are supported.")
    endif()

    # For Humble, build lock_free_events_queue via add_subdirectory
    if(rclcpp_VERSION_MAJOR EQUAL 16)
      add_subdirectory("${irobot_events_executor_SOURCE_DIR}/irobot_lock_free_events_queue" "${irobot_events_executor_BINARY_DIR}_lock_free_queue")

      if(TARGET irobot_lock_free_events_queue)
        if(NOT TARGET irobot_events_executor::irobot_lock_free_events_queue)
          add_library(irobot_events_executor::irobot_lock_free_events_queue ALIAS irobot_lock_free_events_queue)
        endif()
      endif()
    endif()

  endif()
endfunction()

get_events_executor()
