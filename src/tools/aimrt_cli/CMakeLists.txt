# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

# Get all the files in folder
file(GLOB_RECURSE AIMRT_CLI_SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/aimrt_cli/*")

add_custom_target(
  aimrt_cli ALL
  COMMAND ${PYTHON_EXECUTABLE} -m PyInstaller ${CMAKE_CURRENT_SOURCE_DIR}/aimrt_cli/aimrt_cli.spec
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "${PYTHON_EXECUTABLE} -m PyInstaller ${CMAKE_CURRENT_SOURCE_DIR}/aimrt_cli/aimrt_cli.spec"
  DEPENDS ${AIMRT_CLI_SRC_FILES})

if(AIMRT_INSTALL)
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dist/
    DESTINATION bin
    USE_SOURCE_PERMISSIONS)
endif()
