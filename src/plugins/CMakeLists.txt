# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

set_namespace()

if(AIMRT_BUILD_NET_PLUGIN)
  add_subdirectory(net_plugin)
endif()

if(AIMRT_BUILD_WITH_ROS2 AND AIMRT_BUILD_ROS2_PLUGIN)
  add_subdirectory(ros2_plugin)
endif()

if(AIMRT_BUILD_MQTT_PLUGIN)
  add_subdirectory(mqtt_plugin)
endif()

if(AIMRT_BUILD_ZENOH_PLUGIN)
  add_subdirectory(zenoh_plugin)
endif()

if(AIMRT_BUILD_ICEORYX_PLUGIN)
  add_subdirectory(iceoryx_plugin)
endif()

if(AIMRT_BUILD_WITH_PROTOBUF AND AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN)
  add_subdirectory(record_playback_plugin)
endif()

if(AIMRT_BUILD_WITH_PROTOBUF AND AIMRT_BUILD_TIME_MANIPULATOR_PLUGIN)
  add_subdirectory(time_manipulator_plugin)
endif()

if(AIMRT_BUILD_WITH_PROTOBUF AND AIMRT_BUILD_PARAMETER_PLUGIN)
  add_subdirectory(parameter_plugin)
endif()

if(AIMRT_BUILD_WITH_PROTOBUF AND AIMRT_BUILD_LOG_CONTROL_PLUGIN)
  add_subdirectory(log_control_plugin)
endif()

if(AIMRT_BUILD_OPENTELEMETRY_PLUGIN)
  add_subdirectory(opentelemetry_plugin)
endif()

if(AIMRT_BUILD_WITH_PROTOBUF AND AIMRT_BUILD_GRPC_PLUGIN)
  add_subdirectory(grpc_plugin)
endif()

if(AIMRT_BUILD_WITH_PROTOBUF AND AIMRT_BUILD_ECHO_PLUGIN)
  add_subdirectory(echo_plugin)
endif()
