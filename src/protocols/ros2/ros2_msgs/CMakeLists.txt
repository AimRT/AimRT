# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.24)

# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Get namespace
get_namespace(CUR_SUPERIOR_NAMESPACE)
string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})

# Set target name
set(CUR_PACKAGE_NAME ${CUR_DIR})
set(CUR_TARGET_NAME ${CUR_SUPERIOR_NAMESPACE_UNDERLINE}_${CUR_DIR})
set(CUR_TARGET_ALIAS_NAME ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR})

project(${CUR_PACKAGE_NAME})

find_package(action_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pendulum_msgs REQUIRED)
find_package(plotjuggler_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(test_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(statistics_msgs REQUIRED)

# Type support pkg
add_library(${CUR_TARGET_NAME}_ts SHARED ${CMAKE_CURRENT_SOURCE_DIR}/type_support_pkg_main.cc)
add_library(${CUR_TARGET_ALIAS_NAME}_ts ALIAS ${CUR_TARGET_NAME}_ts)
target_link_libraries(
  ${CUR_TARGET_NAME}_ts
  PRIVATE aimrt::interface::aimrt_type_support_pkg_c_interface
          aimrt::interface::aimrt_module_ros2_interface
          action_msgs::action_msgs__rosidl_generator_cpp
          action_msgs::action_msgs__rosidl_typesupport_cpp
          actionlib_msgs::actionlib_msgs__rosidl_generator_cpp
          actionlib_msgs::actionlib_msgs__rosidl_typesupport_cpp
          actuator_msgs::actuator_msgs__rosidl_generator_cpp
          actuator_msgs::actuator_msgs__rosidl_typesupport_cpp
          diagnostic_msgs::diagnostic_msgs__rosidl_generator_cpp
          diagnostic_msgs::diagnostic_msgs__rosidl_typesupport_cpp
          geometry_msgs::geometry_msgs__rosidl_generator_cpp
          geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
          grid_map_msgs::grid_map_msgs__rosidl_generator_cpp
          grid_map_msgs::grid_map_msgs__rosidl_typesupport_cpp
          lifecycle_msgs::lifecycle_msgs__rosidl_generator_cpp
          lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_cpp
          map_msgs::map_msgs__rosidl_generator_cpp
          map_msgs::map_msgs__rosidl_typesupport_cpp
          nav2_msgs::nav2_msgs__rosidl_generator_cpp
          nav2_msgs::nav2_msgs__rosidl_typesupport_cpp
          nav_msgs::nav_msgs__rosidl_generator_cpp
          nav_msgs::nav_msgs__rosidl_typesupport_cpp
          octomap_msgs::octomap_msgs__rosidl_generator_cpp
          octomap_msgs::octomap_msgs__rosidl_typesupport_cpp
          pcl_msgs::pcl_msgs__rosidl_generator_cpp
          pcl_msgs::pcl_msgs__rosidl_typesupport_cpp
          pendulum_msgs::pendulum_msgs__rosidl_generator_cpp
          pendulum_msgs::pendulum_msgs__rosidl_typesupport_cpp
          plotjuggler_msgs::plotjuggler_msgs__rosidl_generator_cpp
          plotjuggler_msgs::plotjuggler_msgs__rosidl_typesupport_cpp
          std_msgs::std_msgs__rosidl_generator_cpp
          std_msgs::std_msgs__rosidl_typesupport_cpp
          stereo_msgs::stereo_msgs__rosidl_generator_cpp
          stereo_msgs::stereo_msgs__rosidl_typesupport_cpp
          test_msgs::test_msgs__rosidl_generator_cpp
          test_msgs::test_msgs__rosidl_typesupport_cpp
          tf2_msgs::tf2_msgs__rosidl_generator_cpp
          tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
          trajectory_msgs::trajectory_msgs__rosidl_generator_cpp
          trajectory_msgs::trajectory_msgs__rosidl_typesupport_cpp
          vision_msgs::vision_msgs__rosidl_generator_cpp
          vision_msgs::vision_msgs__rosidl_typesupport_cpp
          visualization_msgs::visualization_msgs__rosidl_generator_cpp
          visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
          sensor_msgs::sensor_msgs__rosidl_generator_cpp
          sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
          shape_msgs::shape_msgs__rosidl_generator_cpp
          shape_msgs::shape_msgs__rosidl_typesupport_cpp)

set_target_properties(${CUR_TARGET_NAME}_ts PROPERTIES OUTPUT_NAME ${CUR_DIR}_ros2_ts)

if(AIMRT_INSTALL)
  install(TARGETS ${CUR_TARGET_NAME}_ts LIBRARY DESTINATION bin)
endif()
