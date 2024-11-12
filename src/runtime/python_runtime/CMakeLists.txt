# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Get namespace
get_namespace(CUR_SUPERIOR_NAMESPACE)
string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})

# Set target name
set(CUR_TARGET_NAME ${CUR_SUPERIOR_NAMESPACE_UNDERLINE}_${CUR_DIR})
set(CUR_TARGET_ALIAS_NAME ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR})

# Set file collection
file(GLOB_RECURSE src CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)
file(GLOB_RECURSE py_files CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.py)

# Add target
pybind11_add_module(${CUR_TARGET_NAME} SHARED)
add_library(${CUR_TARGET_ALIAS_NAME} ALIAS ${CUR_TARGET_NAME})

# Set source file of target
target_sources(${CUR_TARGET_NAME} PRIVATE ${src})

# Set link libraries of target
target_link_libraries(
  ${CUR_TARGET_NAME}
  PRIVATE $<LINK_LIBRARY:WHOLE_ARCHIVE,aimrt::runtime::core>)

# Link ros2 interface if building with ros2
if(AIMRT_BUILD_WITH_ROS2)
  target_link_libraries(${CUR_TARGET_NAME} PRIVATE aimrt::interface::aimrt_module_ros2_interface)
  target_compile_definitions(${CUR_TARGET_NAME} PRIVATE AIMRT_BUILD_WITH_ROS2)
endif()

# Set misc of target
set_target_properties(${CUR_TARGET_NAME} PROPERTIES OUTPUT_NAME "aimrt_python_runtime")

set_property(TARGET ${CUR_TARGET_NAME} PROPERTY PY_FILES ${py_files})
