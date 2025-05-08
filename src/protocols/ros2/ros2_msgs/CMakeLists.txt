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

set(ROS2_MESSAGE_PACKAGES
    action_msgs
    actionlib_msgs
    actuator_msgs
    diagnostic_msgs
    geometry_msgs
    grid_map_msgs
    lifecycle_msgs
    map_msgs
    nav2_msgs
    nav_msgs
    octomap_msgs
    pcl_msgs
    pendulum_msgs
    plotjuggler_msgs
    std_msgs
    stereo_msgs
    test_msgs
    tf2_msgs
    trajectory_msgs
    unique_identifier_msgs
    sensor_msgs
    visualization_msgs
    shape_msgs
    vision_msgs
    statistics_msgs)

# Type support pkg
add_library(${CUR_TARGET_NAME}_ts SHARED ${CMAKE_CURRENT_SOURCE_DIR}/type_support_pkg_main.cc)
add_library(${CUR_TARGET_ALIAS_NAME}_ts ALIAS ${CUR_TARGET_NAME}_ts)

target_link_libraries(
  ${CUR_TARGET_NAME}_ts
  PRIVATE aimrt::interface::aimrt_type_support_pkg_c_interface
          aimrt::interface::aimrt_module_ros2_interface)
set(EXTRA_LINK_LIBRARIES "")

foreach(PKG_NAME ${ROS2_MESSAGE_PACKAGES})
  string(TOUPPER ${PKG_NAME} PKG_NAME_UPPER)
  set(COMPILE_DEFINITION "AIMRT_ROS2_MSG_${PKG_NAME_UPPER}") # e.g., AIMRT_ROS2_MSG_STD_MSGS

  find_package(${PKG_NAME})

  if(${PKG_NAME}_FOUND)
    message(STATUS "Found ROS 2 message package: ${PKG_NAME}")
    target_compile_definitions(${CUR_TARGET_NAME}_ts PRIVATE ${COMPILE_DEFINITION})
    list(APPEND EXTRA_LINK_LIBRARIES ${PKG_NAME}::${PKG_NAME}__rosidl_generator_cpp ${PKG_NAME}::${PKG_NAME}__rosidl_typesupport_cpp)
  else()
    message(STATUS "ROS 2 message package NOT found: ${PKG_NAME}. Corresponding types will not be included.")
  endif()
endforeach()

if(EXTRA_LINK_LIBRARIES)
  target_link_libraries(${CUR_TARGET_NAME}_ts PRIVATE ${EXTRA_LINK_LIBRARIES})
endif()

set_target_properties(${CUR_TARGET_NAME}_ts PROPERTIES OUTPUT_NAME ${CUR_DIR}_ros2_ts)

if(AIMRT_INSTALL)
  install(TARGETS ${CUR_TARGET_NAME}_ts LIBRARY DESTINATION bin)
endif()
