# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

# add target for aimrt rpc gen code target for one file
function(add_ros2_aimrt_rpc_gencode_target_for_one_file)
  cmake_parse_arguments(ARG "" "TARGET_NAME" "PACKAGE_NAME;PROTO_FILE;GENCODE_PATH;DEP_PROTO_TARGETS;OPTIONS" ${ARGN})

  if(NOT EXISTS ${ARG_GENCODE_PATH})
    file(MAKE_DIRECTORY ${ARG_GENCODE_PATH})
  endif()

  string(REGEX REPLACE ".+/(.+)\\..*" "\\1" PROTO_FILE_NAME ${ARG_PROTO_FILE})
  string(REGEX REPLACE "(.+)/(.+)\\..*" "\\1" PROTO_PATH ${ARG_PROTO_FILE})
  set(GEN_SRC "${ARG_GENCODE_PATH}/${PROTO_FILE_NAME}.aimrt_rpc.srv.cc")
  set(GEN_HDR "${ARG_GENCODE_PATH}/${PROTO_FILE_NAME}.aimrt_rpc.srv.h")

  get_property(ROS2_AIMRT_RPC_GEN_CODE_TOOL_PATH GLOBAL PROPERTY ROS2_AIMRT_RPC_GEN_CODE_TOOL_PATH_PROPERTY)

  add_custom_command(
    OUTPUT ${GEN_SRC} ${GEN_HDR}
    COMMAND Python3::Interpreter ARGS ${ROS2_AIMRT_RPC_GEN_CODE_TOOL_PATH}/ros2_py_gen_aimrt_cpp_rpc.py --pkg_name=${ARG_PACKAGE_NAME} --srv_file=${ARG_PROTO_FILE}
            --output_path=${ARG_GENCODE_PATH} ${ARG_OPTIONS}
    DEPENDS ${ARG_PROTO_FILE} ${ROS2_AIMRT_RPC_GEN_CODE_TOOL_PATH}/ros2_py_gen_aimrt_cpp_rpc.py
    COMMENT "Generating aimrt rpc code for ROS interfaces"
    VERBATIM)

  add_library(${ARG_TARGET_NAME} STATIC)

  target_sources(${ARG_TARGET_NAME} PRIVATE ${GEN_SRC})
  target_sources(${ARG_TARGET_NAME} PUBLIC FILE_SET HEADERS BASE_DIRS ${ARG_GENCODE_PATH} FILES ${GEN_HDR})

  target_include_directories(${ARG_TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${ARG_GENCODE_PATH}> $<INSTALL_INTERFACE:include/${ARG_TARGET_NAME}>)

  target_link_libraries(${ARG_TARGET_NAME} PUBLIC aimrt::interface::aimrt_module_ros2_interface ${ARG_DEP_PROTO_TARGETS})
endfunction()
