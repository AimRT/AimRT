# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

set_namespace()

add_subdirectory(helloworld)
add_subdirectory(logger)
add_subdirectory(executor)
add_subdirectory(context)
add_subdirectory(parameter)

if(AIMRT_BUILD_WITH_PROTOBUF)
  add_subdirectory(pb_chn)
  add_subdirectory(pb_rpc)
endif()

if(AIMRT_BUILD_WITH_ROS2)
  add_subdirectory(ros2_chn)
  add_subdirectory(ros2_rpc)
endif()
