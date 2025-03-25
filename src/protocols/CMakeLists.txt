# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

set_namespace()

if(AIMRT_BUILD_WITH_PROTOBUF)
  add_subdirectory(protobuf/common)
  add_subdirectory(protobuf/geometry)
  add_subdirectory(protobuf/sensor)

  if(AIMRT_BUILD_EXAMPLES)
    add_subdirectory(protobuf/example)
  endif()

  if(AIMRT_BUILD_TIME_MANIPULATOR_PLUGIN)
    add_subdirectory(plugins/time_manipulator_plugin)
  endif()

  if(AIMRT_BUILD_PARAMETER_PLUGIN)
    add_subdirectory(plugins/parameter_plugin)
  endif()

  if(AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN)
    add_subdirectory(plugins/record_playback_plugin)
  endif()

  if(AIMRT_BUILD_LOG_CONTROL_PLUGIN)
    add_subdirectory(plugins/log_control_plugin)
  endif()

  if(AIMRT_BUILD_TOPIC_LOGGER_PLUGIN)
    add_subdirectory(plugins/topic_logger_plugin)
  endif()

endif()

if(AIMRT_BUILD_WITH_ROS2)
  add_subdirectory(ros2/sensor)

  if(AIMRT_BUILD_EXAMPLES)
    add_subdirectory(ros2/example_ros2)
  endif()

  if(AIMRT_BUILD_ROS2_PLUGIN)
    add_subdirectory(plugins/ros2_plugin_proto)
  endif()
endif()