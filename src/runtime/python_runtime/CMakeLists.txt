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
file(GLOB_RECURSE py_files CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.py)
file(GLOB_RECURSE pyi_files CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.pyi)
list(APPEND pyi_files ${CMAKE_CURRENT_SOURCE_DIR}/py.typed)

# Add target
pybind11_add_module(${CUR_TARGET_NAME} SHARED)
add_library(${CUR_TARGET_ALIAS_NAME} ALIAS ${CUR_TARGET_NAME})

# Set source file of target
target_sources(${CUR_TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/python_runtime_main.cc)

# Set link libraries of target
target_link_libraries(
  ${CUR_TARGET_NAME}
  PRIVATE $<LINK_LIBRARY:WHOLE_ARCHIVE,aimrt::runtime::core>)

# Set misc of target
set_target_properties(${CUR_TARGET_NAME} PROPERTIES OUTPUT_NAME "aimrt_python_runtime")

set_property(TARGET ${CUR_TARGET_NAME} PROPERTY PY_FILES ${py_files})
set_property(TARGET ${CUR_TARGET_NAME} PROPERTY PYI_FILES ${pyi_files})

# Ros2 support
if(AIMRT_BUILD_WITH_ROS2)
  pybind11_add_module(${CUR_TARGET_NAME}_ros2 SHARED)
  add_library(${CUR_TARGET_ALIAS_NAME}_ros2 ALIAS ${CUR_TARGET_NAME}_ros2)

  target_sources(${CUR_TARGET_NAME}_ros2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ros2_type_support_utils.cc ${CMAKE_CURRENT_SOURCE_DIR}/python_runtime_ros2.cc)

  target_link_libraries(
    ${CUR_TARGET_NAME}_ros2 #
    PRIVATE $<LINK_LIBRARY:WHOLE_ARCHIVE,aimrt::runtime::core> #
            aimrt::interface::aimrt_module_ros2_interface)

  set_target_properties(${CUR_TARGET_NAME}_ros2 PROPERTIES OUTPUT_NAME "aimrt_python_runtime_ros2")
endif()
