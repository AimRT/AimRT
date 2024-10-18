# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

# Get all the files in folder
file(GLOB_RECURSE AIMRT_CLI_SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/aimrt_cli/*")

set(AIMRT_CLI_BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

add_custom_target(
  aimrt_cli 
  COMMAND ${PYTHON_EXECUTABLE} -m PyInstaller ${CMAKE_CURRENT_SOURCE_DIR}/aimrt_cli/aimrt_cli.spec
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "${PYTHON_EXECUTABLE} -m PyInstaller ${CMAKE_CURRENT_SOURCE_DIR}/aimrt_cli/aimrt_cli.spec"
  DEPENDS ${AIMRT_CLI_SRC_FILES})
  
add_custom_target(
  copy_ros2_plugin_proto_files_aimrt_cli
  COMMAND ${CMAKE_COMMAND} -E make_directory ${AIMRT_CLI_BUILD_DIR}/lib/
  COMMAND ${CMAKE_COMMAND} -E make_directory ${AIMRT_CLI_BUILD_DIR}/local/
  COMMAND ${CMAKE_COMMAND} -E make_directory ${AIMRT_CLI_BUILD_DIR}/share/
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_INSTALL_PREFIX}/lib/ ${AIMRT_CLI_BUILD_DIR}/lib/
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_INSTALL_PREFIX}/local/ ${AIMRT_CLI_BUILD_DIR}/local/
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_INSTALL_PREFIX}/share/ ${AIMRT_CLI_BUILD_DIR}/share/
  COMMENT "Copying ROS2 files"
  DEPENDS aimrt::plugins::ros2_plugin)

if(AIMRT_BUILD_WITH_ROS2 AND AIMRT_BUILD_ROS2_PLUGIN)
  add_dependencies(aimrt_cli copy_ros2_plugin_proto_files_aimrt_cli)
endif()

if(AIMRT_INSTALL)
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION bin
    USE_SOURCE_PERMISSIONS)
endif()
