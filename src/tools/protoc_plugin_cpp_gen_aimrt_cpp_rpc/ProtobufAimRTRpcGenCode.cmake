# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

# add target for aimrt rpc gen code target for proto files
function(add_protobuf_aimrt_rpc_gencode_target_for_proto_files)
  cmake_parse_arguments(ARG "" "TARGET_NAME" "PROTO_FILES;GENCODE_PATH;DEP_PROTO_TARGETS;OPTIONS" ${ARGN})

  if(NOT EXISTS ${ARG_GENCODE_PATH})
    file(MAKE_DIRECTORY ${ARG_GENCODE_PATH})
  endif()

  set(ALL_DEP_PROTO_TARGETS ${ARG_DEP_PROTO_TARGETS})
  foreach(CUR_DEP_PROTO_TARGET ${ARG_DEP_PROTO_TARGETS})
    get_target_property(SECONDARY_DEP_PROTO_TARGET ${CUR_DEP_PROTO_TARGET} DEP_PROTO_TARGETS)
    list(APPEND ALL_DEP_PROTO_TARGETS ${SECONDARY_DEP_PROTO_TARGET})
  endforeach()
  list(REMOVE_DUPLICATES ALL_DEP_PROTO_TARGETS)

  set(PROTOC_EXTERNAL_PROTO_PATH_ARGS)
  foreach(CUR_DEP_PROTO_TARGET ${ALL_DEP_PROTO_TARGETS})
    get_target_property(DEP_PROTO_TARGET_PROTO_PATH ${CUR_DEP_PROTO_TARGET} PROTO_PATH)
    list(APPEND PROTOC_EXTERNAL_PROTO_PATH_ARGS "--proto_path=${DEP_PROTO_TARGET_PROTO_PATH}")
  endforeach()
  list(REMOVE_DUPLICATES PROTOC_EXTERNAL_PROTO_PATH_ARGS)

  get_property(PROTOC_NAMESPACE GLOBAL PROPERTY PROTOC_NAMESPACE_PROPERTY)

  if(NOT PROTOC_NAMESPACE)
    set(CUR_PROTOC_NAMESPACE "protobuf")
  else()
    set(CUR_PROTOC_NAMESPACE ${PROTOC_NAMESPACE})
  endif()

  set(GEN_SRCS)
  set(GEN_HDRS)

  foreach(PROTO_FILE ${ARG_PROTO_FILES})
    string(REGEX REPLACE ".+/(.+)\\..*" "\\1" PROTO_FILE_NAME ${PROTO_FILE})
    string(REGEX REPLACE "(.+)/(.+)\\..*" "\\1" PROTO_FILE_PATH ${PROTO_FILE})
    set(GEN_SRC "${ARG_GENCODE_PATH}/${PROTO_FILE_NAME}.aimrt_rpc.pb.cc")
    set(GEN_HDR "${ARG_GENCODE_PATH}/${PROTO_FILE_NAME}.aimrt_rpc.pb.h")

    list(APPEND GEN_SRCS ${GEN_SRC})
    list(APPEND GEN_HDRS ${GEN_HDR})

    add_custom_command(
      OUTPUT ${GEN_SRC} ${GEN_HDR}
      COMMAND ${CUR_PROTOC_NAMESPACE}::protoc ARGS ${ARG_OPTIONS} --proto_path ${PROTO_FILE_PATH} ${PROTOC_EXTERNAL_PROTO_PATH_ARGS} --aimrt_rpc_out ${ARG_GENCODE_PATH}
              --plugin=protoc-gen-aimrt_rpc=$<TARGET_FILE:aimrt::tools::protoc_plugin_cpp_gen_aimrt_cpp_rpc> ${PROTO_FILE}
      DEPENDS ${PROTO_FILE} ${CUR_PROTOC_NAMESPACE}::protoc aimrt::tools::protoc_plugin_cpp_gen_aimrt_cpp_rpc
      COMMENT
        "Running protoc, args: ${ARG_OPTIONS} --proto_path ${PROTO_FILE_PATH} ${PROTOC_EXTERNAL_PROTO_PATH_ARGS} --aimrt_rpc_out ${ARG_GENCODE_PATH} --plugin=protoc-gen-aimrt_rpc=$<TARGET_FILE:aimrt::tools::protoc_plugin_cpp_gen_aimrt_cpp_rpc> ${PROTO_FILE}"
      VERBATIM)
  endforeach()

  add_library(${ARG_TARGET_NAME} STATIC)

  target_sources(${ARG_TARGET_NAME} PRIVATE ${GEN_SRCS})
  target_sources(${ARG_TARGET_NAME} PUBLIC FILE_SET HEADERS BASE_DIRS ${ARG_GENCODE_PATH} FILES ${GEN_HDRS})

  target_include_directories(${ARG_TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${ARG_GENCODE_PATH}> $<INSTALL_INTERFACE:include/${ARG_TARGET_NAME}>)

  target_link_libraries(${ARG_TARGET_NAME} PUBLIC aimrt::interface::aimrt_module_protobuf_interface ${ALL_DEP_PROTO_TARGETS})

endfunction()
