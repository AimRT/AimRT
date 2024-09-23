# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

set_property(GLOBAL PROPERTY ROS2_AIMRT_RPC_GEN_CODE_TOOL_PATH_PROPERTY "${CMAKE_CURRENT_SOURCE_DIR}")
include(${CMAKE_CURRENT_SOURCE_DIR}/Ros2AimRTRpcGenCode.cmake)
if(AIMRT_INSTALL)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/Ros2AimRTRpcGenCode.cmake DESTINATION cmake)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/ros2_py_gen_aimrt_cpp_rpc.py DESTINATION bin)
endif()
