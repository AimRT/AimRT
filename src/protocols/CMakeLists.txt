# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

set_namespace()

if(AIMRT_BUILD_WITH_PROTOBUF)
  add_subdirectory(common)
  add_subdirectory(sensor)

  if(AIMRT_BUILD_EXAMPLES)
    add_subdirectory(example)
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

endif()

if(AIMRT_BUILD_WITH_ROS2)
  if(AIMRT_BUILD_EXAMPLES)
    add_subdirectory(example_ros2)
  endif()

  if(AIMRT_BUILD_ROS2_PLUGIN OR AIMRT_BUILD_BAGTRANS)
    add_subdirectory(plugins/ros2_plugin_proto)
  endif()
endif()
