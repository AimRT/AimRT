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

# Add target
add_protobuf_gencode_target_for_proto_path(
  TARGET_NAME ${CUR_TARGET_NAME}_pb_gencode
  PROTO_PATH ${CMAKE_CURRENT_SOURCE_DIR}
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR})
add_library(${CUR_TARGET_ALIAS_NAME}_pb_gencode ALIAS ${CUR_TARGET_NAME}_pb_gencode)

# Type support pkg
add_library(${CUR_TARGET_NAME}_ts SHARED ${CMAKE_CURRENT_SOURCE_DIR}/type_support_pkg_main.cc)
add_library(${CUR_TARGET_ALIAS_NAME}_ts ALIAS ${CUR_TARGET_NAME}_ts)
target_link_libraries(
  ${CUR_TARGET_NAME}_ts
  PRIVATE aimrt::interface::aimrt_type_support_pkg_c_interface
          aimrt::interface::aimrt_module_protobuf_interface
          ${CUR_TARGET_ALIAS_NAME}_pb_gencode)
set_target_properties(${CUR_TARGET_NAME}_ts PROPERTIES OUTPUT_NAME ${CUR_DIR}_pb_ts)

# Set installation
if(AIMRT_INSTALL)
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DESTINATION "share"
    FILES_MATCHING
    PATTERN "*.proto")

  set_property(TARGET ${CUR_TARGET_NAME}_pb_gencode PROPERTY EXPORT_NAME ${CUR_TARGET_ALIAS_NAME}_pb_gencode)
  install(
    TARGETS ${CUR_TARGET_NAME}_pb_gencode
    EXPORT ${INSTALL_CONFIG_NAME}
    ARCHIVE DESTINATION lib
            FILE_SET HEADERS
            DESTINATION include/${CUR_TARGET_NAME}_pb_gencode)
  install(TARGETS ${CUR_TARGET_NAME}_ts LIBRARY DESTINATION bin)
endif()
