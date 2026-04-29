# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Get namespace
get_namespace(CUR_SUPERIOR_NAMESPACE)
string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})

# Set target name
set(CUR_TARGET_NAME ${CUR_SUPERIOR_NAMESPACE_UNDERLINE}_agi_header_plugin_example_app)
set(CUR_TARGET_ALIAS_NAME ${CUR_SUPERIOR_NAMESPACE}::agi_header_plugin_example_app)

# Set file collection
file(GLOB_RECURSE src ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

# Add target
add_executable(${CUR_TARGET_NAME})
add_executable(${CUR_TARGET_ALIAS_NAME} ALIAS ${CUR_TARGET_NAME})

# Set source file of target
target_sources(${CUR_TARGET_NAME} PRIVATE ${src})

# Set include path of target
target_include_directories(
  ${CUR_TARGET_NAME}
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

# Set link libraries of target
target_link_libraries(
  ${CUR_TARGET_NAME}
  PRIVATE aimrt::runtime::core)

if(AIMRT_BUILD_WITH_PROTOBUF)
  target_link_libraries(${CUR_TARGET_NAME} PRIVATE aimrt::interface::aimrt_module_protobuf_interface ${CUR_SUPERIOR_NAMESPACE}::proto_pb_gencode)
  target_compile_definitions(${CUR_TARGET_NAME} PRIVATE AIMRT_BUILD_WITH_PROTOBUF)
endif()

if(AIMRT_BUILD_WITH_ROS2)
  target_link_libraries(${CUR_TARGET_NAME} PRIVATE aimrt::interface::aimrt_module_ros2_interface rclcpp::rclcpp ros2_msg::ros2_msg__rosidl_generator_cpp
                                                   ros2_msg::ros2_msg__rosidl_typesupport_cpp ros2_msg::ros2_msg__rosidl_typesupport_fastrtps_cpp)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(${CUR_TARGET_NAME} PRIVATE -Wl,--no-as-needed ros2_msg::ros2_msg__rosidl_typesupport_introspection_cpp -Wl,--as-needed)
  else()
    target_link_libraries(${CUR_TARGET_NAME} PRIVATE ros2_msg::ros2_msg__rosidl_typesupport_introspection_cpp)
  endif()
  target_compile_definitions(${CUR_TARGET_NAME} PRIVATE AIMRT_BUILD_WITH_ROS2)
endif()

# Set misc of target
set_target_properties(
  ${CUR_TARGET_NAME}
  PROPERTIES OUTPUT_NAME "agi_header_plugin_example_app"
             BUILD_RPATH "$ORIGIN"
             INSTALL_RPATH "$ORIGIN")
