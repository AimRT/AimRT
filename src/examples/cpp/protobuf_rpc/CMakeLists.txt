# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Set namespace
set_namespace()

# module
add_subdirectory(module/benchmark_rpc_client_module)
add_subdirectory(module/normal_rpc_co_client_module)
add_subdirectory(module/normal_rpc_sync_client_module)
add_subdirectory(module/normal_rpc_async_client_module)
add_subdirectory(module/normal_rpc_future_client_module)
add_subdirectory(module/normal_rpc_co_server_module)
add_subdirectory(module/normal_rpc_sync_server_module)
add_subdirectory(module/normal_rpc_async_server_module)

# pkg
add_subdirectory(pkg/protobuf_rpc_pkg)
add_subdirectory(pkg/protobuf_rpc_client_pkg)
add_subdirectory(pkg/protobuf_rpc_server_pkg)

# install
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(CUR_INSTALL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/install/linux)
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
  set(CUR_INSTALL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/install/win)
else()
  message(FATAL_ERROR "Unsupport os")
endif()

# build all
get_namespace(CUR_SUPERIOR_NAMESPACE)
string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})
add_custom_target(
  ${CUR_SUPERIOR_NAMESPACE_UNDERLINE}_${CUR_DIR}_build_all ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CUR_INSTALL_SOURCE_DIR}/bin ${CMAKE_BINARY_DIR}
  DEPENDS aimrt::runtime::main
          ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR}::protobuf_rpc_pkg
          ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR}::protobuf_rpc_client_pkg
          ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR}::protobuf_rpc_server_pkg)
