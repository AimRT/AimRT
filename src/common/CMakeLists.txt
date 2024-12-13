# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

set_namespace()

add_subdirectory(util)

if(AIMRT_BUILD_WITH_ROS2)
  add_subdirectory(ros2_util)
endif()

if(AIMRT_BUILD_NET_PLUGIN OR AIMRT_BUILD_GRPC_PLUGIN)
  add_subdirectory(net)
endif()
