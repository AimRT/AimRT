# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

if(AIMRT_USE_PROTOC_PYTHON_PLUGIN)
  set_property(GLOBAL PROPERTY PROTOC_PLUGIN_PY_GEN_AIMRT_CPP_RPC_PATH_PROPERTY "${CMAKE_CURRENT_SOURCE_DIR}")
  include(${CMAKE_CURRENT_SOURCE_DIR}/ProtobufAimRTRpcGenCode.cmake)
  if(AIMRT_INSTALL)
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/ProtobufAimRTRpcGenCode.cmake DESTINATION cmake)
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/protoc_plugin_py_gen_aimrt_cpp_rpc.py DESTINATION bin)
  endif()
endif()
