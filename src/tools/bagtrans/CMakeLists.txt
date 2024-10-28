# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

# Get all the files in folder
file(GLOB_RECURSE BAGTRANS_SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/bagtrans/*")

set(BAGTRANS_BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

add_custom_target(
  bagtrans
  COMMAND ${Python3_EXECUTABLE} -m build --wheel --no-isolation -v
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "${PYTHON_EXECUTABLE} -m build --wheel --no-isolation -v"
  DEPENDS ${BAGTRANS_SRC_FILES})

add_custom_target(
  copy_ros2_plugin_proto_files_bagtrans
  COMMAND ${CMAKE_COMMAND} -E make_directory ${BAGTRANS_BUILD_DIR}/lib/
  COMMAND ${CMAKE_COMMAND} -E make_directory ${BAGTRANS_BUILD_DIR}/local/
  COMMAND ${CMAKE_COMMAND} -E make_directory ${BAGTRANS_BUILD_DIR}/share/
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_INSTALL_PREFIX}/lib/ ${BAGTRANS_BUILD_DIR}/lib/
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_INSTALL_PREFIX}/local/ ${BAGTRANS_BUILD_DIR}/local/
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_INSTALL_PREFIX}/share/ ${BAGTRANS_BUILD_DIR}/share/
  COMMENT "Copying ROS2 files"
  DEPENDS aimrt::plugins::ros2_plugin)

add_dependencies(bagtrans copy_ros2_plugin_proto_files_bagtrans)

if(AIMRT_INSTALL)
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DESTINATION bin
    USE_SOURCE_PERMISSIONS)
endif()
