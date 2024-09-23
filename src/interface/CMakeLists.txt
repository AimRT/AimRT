# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

set_namespace()

add_subdirectory(aimrt_module_c_interface)
add_subdirectory(aimrt_module_cpp_interface)
add_subdirectory(aimrt_pkg_c_interface)
add_subdirectory(aimrt_type_support_pkg_c_interface)

if(AIMRT_BUILD_WITH_PROTOBUF)
  add_subdirectory(aimrt_module_protobuf_interface)
endif()

if(AIMRT_BUILD_WITH_ROS2)
  add_subdirectory(aimrt_module_ros2_interface)
endif()

if(AIMRT_BUILD_RUNTIME)
  add_subdirectory(aimrt_core_plugin_interface)
endif()
