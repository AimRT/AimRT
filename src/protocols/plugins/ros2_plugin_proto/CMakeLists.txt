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

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# BUG of ros2
set(CUR_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS ON)
# cmake-format: off
rosidl_generate_interfaces(${CUR_PACKAGE_NAME}
  "msg/RosMsgWrapper.msg"
  "srv/RosRpcWrapper.srv"
)
# cmake-format: on
set(BUILD_SHARED_LIBS ${CUR_BUILD_SHARED_LIBS})

if(NOT TARGET ${CUR_PACKAGE_NAME}::${CUR_PACKAGE_NAME}__rosidl_typesupport_cpp)
  add_library(${CUR_PACKAGE_NAME}::${CUR_PACKAGE_NAME}__rosidl_typesupport_cpp ALIAS ${CUR_PACKAGE_NAME}__rosidl_typesupport_cpp)
endif()

if(NOT TARGET ${CUR_PACKAGE_NAME}::${CUR_PACKAGE_NAME}__rosidl_typesupport_fastrtps_cpp)
  add_library(${CUR_PACKAGE_NAME}::${CUR_PACKAGE_NAME}__rosidl_typesupport_fastrtps_cpp ALIAS ${CUR_PACKAGE_NAME}__rosidl_typesupport_fastrtps_cpp)
endif()

add_ros2_aimrt_rpc_gencode_target_for_one_file(
  TARGET_NAME ${CUR_TARGET_NAME}_aimrt_rpc_gencode
  PACKAGE_NAME ${CUR_PACKAGE_NAME}
  PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/srv/RosRpcWrapper.srv
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS
    rclcpp::rclcpp
    ${CUR_PACKAGE_NAME}::${CUR_PACKAGE_NAME}__rosidl_generator_cpp
    ${CUR_PACKAGE_NAME}::${CUR_PACKAGE_NAME}__rosidl_typesupport_cpp
    ${CUR_PACKAGE_NAME}::${CUR_PACKAGE_NAME}__rosidl_typesupport_fastrtps_cpp
    ${CUR_PACKAGE_NAME}::${CUR_PACKAGE_NAME}__rosidl_typesupport_introspection_cpp)
add_library(${CUR_TARGET_ALIAS_NAME}_aimrt_rpc_gencode ALIAS ${CUR_TARGET_NAME}_aimrt_rpc_gencode)

# Type support pkg
add_library(${CUR_TARGET_NAME}_ts SHARED ${CMAKE_CURRENT_SOURCE_DIR}/type_support_pkg_main.cc)
add_library(${CUR_TARGET_ALIAS_NAME}_ts ALIAS ${CUR_TARGET_NAME}_ts)
target_link_libraries(
  ${CUR_TARGET_NAME}_ts
  PRIVATE aimrt::interface::aimrt_type_support_pkg_c_interface
          aimrt::interface::aimrt_module_ros2_interface
          ${CUR_PACKAGE_NAME}::${CUR_PACKAGE_NAME}__rosidl_generator_cpp
          ${CUR_PACKAGE_NAME}::${CUR_PACKAGE_NAME}__rosidl_typesupport_cpp)
set_target_properties(${CUR_TARGET_NAME}_ts PROPERTIES OUTPUT_NAME ${CUR_DIR}_ros2_ts)

if(AIMRT_INSTALL)
  set_property(TARGET ${CUR_TARGET_NAME}_aimrt_rpc_gencode PROPERTY EXPORT_NAME ${CUR_TARGET_ALIAS_NAME}_aimrt_rpc_gencode)
  install(
    TARGETS ${CUR_TARGET_NAME}_aimrt_rpc_gencode
    EXPORT ${INSTALL_CONFIG_NAME}
    FILE_SET HEADERS
    DESTINATION include/${CUR_TARGET_NAME}_aimrt_rpc_gencode)
  install(TARGETS ${CUR_TARGET_NAME}_ts LIBRARY DESTINATION bin)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
