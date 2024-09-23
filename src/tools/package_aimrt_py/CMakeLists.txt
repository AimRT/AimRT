# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

get_target_property(py_files aimrt::runtime::python_runtime PY_FILES)
file(GLOB_RECURSE gencode_file ${CMAKE_SOURCE_DIR}/src/tools/protoc_plugin_py_gen_aimrt_py_rpc/*.py) # TODO
list(APPEND py_files ${gencode_file})

set(AIMRT_PY_PKG_DIR ${CMAKE_BINARY_DIR}/aimrt_py_pkg)

add_custom_target(
  copy_python_runtime
  COMMAND ${CMAKE_COMMAND} -E make_directory ${AIMRT_PY_PKG_DIR}/aimrt_py/
  COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aimrt::runtime::python_runtime> ${AIMRT_PY_PKG_DIR}/aimrt_py/
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${py_files} ${AIMRT_PY_PKG_DIR}/aimrt_py/
  COMMENT "Copying python runtime files"
  DEPENDS aimrt::runtime::python_runtime)

add_custom_target(
  copy_ros2_plugin_proto_files
  COMMAND ${CMAKE_COMMAND} -E make_directory ${AIMRT_PY_PKG_DIR}/aimrt_py/
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_INSTALL_PREFIX}/lib/ ${AIMRT_PY_PKG_DIR}/aimrt_py/lib/
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_INSTALL_PREFIX}/local/ ${AIMRT_PY_PKG_DIR}/aimrt_py/local/
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_INSTALL_PREFIX}/share/ ${AIMRT_PY_PKG_DIR}/aimrt_py/share/
  COMMENT "Copying ROS2 files"
  DEPENDS aimrt::plugins::ros2_plugin)

add_custom_target(
  create_python_pkg
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/pyproject.toml ${CMAKE_CURRENT_SOURCE_DIR}/setup.py ${CMAKE_CURRENT_SOURCE_DIR}/MANIFEST.in
          ${AIMRT_PY_PKG_DIR}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/VERSION ${CMAKE_SOURCE_DIR}/README.md ${CMAKE_SOURCE_DIR}/LICENSE ${AIMRT_PY_PKG_DIR}
  COMMAND ${Python3_EXECUTABLE} -m build --wheel --no-isolation -v
  COMMENT "Creating python package"
  DEPENDS copy_python_runtime
  WORKING_DIRECTORY ${AIMRT_PY_PKG_DIR})

if(AIMRT_BUILD_WITH_ROS2 AND AIMRT_BUILD_ROS2_PLUGIN)
  add_dependencies(create_python_pkg copy_ros2_plugin_proto_files)
endif()

include(PluginDetectTool)
get_subdirectories(${CMAKE_SOURCE_DIR}/src/plugins PLUGIN_LIST)
message(STATUS "Detected plugins: ${PLUGIN_LIST}")

function(copy_plugin_if_enabled PLUGIN_NAME)
  string(TOUPPER ${PLUGIN_NAME} PLUGIN_BUILD_FLAG)
  set(PLUGIN_BUILD_FLAG "AIMRT_BUILD_${PLUGIN_BUILD_FLAG}")

  if(${PLUGIN_BUILD_FLAG})
    add_custom_target(
      copy_${PLUGIN_NAME}
      COMMAND ${CMAKE_COMMAND} -E make_directory ${AIMRT_PY_PKG_DIR}/aimrt_py/
      COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aimrt::plugins::${PLUGIN_NAME}> ${AIMRT_PY_PKG_DIR}/aimrt_py/
      DEPENDS aimrt::plugins::${PLUGIN_NAME})
    add_dependencies(create_python_pkg copy_${PLUGIN_NAME})
  endif()
endfunction()

foreach(PLUGIN_NAME ${PLUGIN_LIST})
  copy_plugin_if_enabled(${PLUGIN_NAME})
endforeach()
