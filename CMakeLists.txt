# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.24)

project(aimrt LANGUAGES C CXX)

# Set cmake path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Some option
option(AIMRT_BUILD_TESTS "AimRT build tests." OFF)
option(AIMRT_BUILD_EXAMPLES "AimRT build examples." OFF)
option(AIMRT_BUILD_DOCUMENT "AimRT build document." OFF)
option(AIMRT_BUILD_RUNTIME "AimRT build runtime." ON)
option(AIMRT_BUILD_CLI_TOOLS "AimRT build aimrt command line tools." OFF)

option(AIMRT_BUILD_PYTHON_RUNTIME "AimRT build python runtime." OFF)

option(AIMRT_USE_FMT_LIB "AimRT use fmt library." ON)

option(AIMRT_BUILD_WITH_PROTOBUF "AimRT build with protobuf." ON)
option(AIMRT_USE_LOCAL_PROTOC_COMPILER "AimRT use local protoc compiler." OFF)
option(AIMRT_USE_PROTOC_PYTHON_PLUGIN "AimRT use protoc python plugin." OFF)

option(AIMRT_BUILD_WITH_ROS2 "AimRT build with ros2." OFF)

option(AIMRT_BUILD_NET_PLUGIN "AimRT build net plugin." OFF)
option(AIMRT_BUILD_ROS2_PLUGIN "AimRT build ros2 plugin." OFF)
option(AIMRT_BUILD_MQTT_PLUGIN "AimRT build mqtt plugin." OFF)
option(AIMRT_BUILD_ZENOH_PLUGIN "AimRT build zenoh plugin." OFF)
option(AIMRT_BUILD_ICEORYX_PLUGIN "AimRT build iceoryx plugin." OFF)
option(AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN "AimRT build record playback plugin." OFF)
option(AIMRT_BUILD_TIME_MANIPULATOR_PLUGIN "AimRT build time manipulator plugin." OFF)
option(AIMRT_BUILD_PARAMETER_PLUGIN "AimRT build parameter plugin." OFF)
option(AIMRT_BUILD_LOG_CONTROL_PLUGIN "AimRT build log control plugin." OFF)
option(AIMRT_BUILD_OPENTELEMETRY_PLUGIN "AimRT build opentelemetry plugin." OFF)
option(AIMRT_BUILD_GRPC_PLUGIN "AimRT build grpc plugin." OFF)

option(AIMRT_INSTALL "Enable installation of AimRT." ON)

option(AIMRT_BUILD_PYTHON_PACKAGE "AimRT build python package." OFF)

option(AIMRT_EXECUTOR_USE_STDEXEC "AimRT use stdexec as executor impl. (Experimental)" OFF)
option(AIMRT_BUILD_WITH_WERROR "AimRT build with -Werror option. (Experimental)" OFF)

# Some necessary settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set default build type as Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

file(READ "${PROJECT_SOURCE_DIR}/VERSION" AIMRT_VERSION)
string(STRIP "${AIMRT_VERSION}" AIMRT_VERSION)

set(INSTALL_CONFIG_NAME ${PROJECT_NAME}-config)

# Master project configuration
if(NOT DEFINED AIMRT_MASTER_PROJECT)
  set(AIMRT_MASTER_PROJECT OFF)
  if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(AIMRT_MASTER_PROJECT ON)
  endif()
endif()

if(AIMRT_MASTER_PROJECT)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

  set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/_deps)

  set(CMAKE_CXX_VISIBILITY_PRESET hidden)
  set(CMAKE_CXX_VISIBLITY_INLINES_HIDDEN ON)
  set(BUILD_SHARED_LIBS OFF)

  if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")

    if(AIMRT_BUILD_TESTS)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
    endif()
  endif()

  if(MSVC)
    add_compile_options(/utf-8 /wd4819)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus")
  endif()

  if(WIN32)
    add_compile_definitions(NOMINMAX)
  endif()

  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# Build document
if(AIMRT_BUILD_DOCUMENT)
  message(STATUS "gen document ...")
  set(AIMRT_DOC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/document)
  if(WIN32)
    execute_process(COMMAND ${AIMRT_DOC_DIR}/doxygen/build.bat ${AIMRT_DOC_DIR}/doxygen WORKING_DIRECTORY ${AIMRT_DOC_DIR}/doxygen)
  elseif(UNIX)
    execute_process(COMMAND ${AIMRT_DOC_DIR}/doxygen/build.sh ${AIMRT_DOC_DIR}/doxygen WORKING_DIRECTORY ${AIMRT_DOC_DIR}/doxygen)
    execute_process(COMMAND ${AIMRT_DOC_DIR}/sphinx-cn/build.sh ${AIMRT_DOC_DIR}/sphinx-cn WORKING_DIRECTORY ${AIMRT_DOC_DIR}/sphinx-cn)
  endif()
endif()

# Include cmake module
include(NamespaceTool)

if(AIMRT_BUILD_TESTS)
  include(GetGTest)
  enable_testing()
endif()

if(AIMRT_EXECUTOR_USE_STDEXEC)
  include(GetStdexec)
else()
  include(GetLibUnifex)
endif()

if(AIMRT_USE_FMT_LIB)
  include(GetFmt)
endif()

if(AIMRT_BUILD_CLI_TOOLS)
  find_package(Python3 REQUIRED)

  include(CheckPythonPackage)
  check_python_package(pyinstaller PYINSTALLER_FOUND)
  check_python_package(jinja2 JINJA2_FOUND)
  check_python_package(pyyaml PYYAML_FOUND)

  if(NOT PYINSTALLER_FOUND
     OR NOT JINJA2_FOUND
     OR NOT PYYAML_FOUND)
    set(AIMRT_BUILD_CLI_TOOLS OFF)
    message(WARNING "Can not find pyinstaller, jinja2 or pyyaml in your python environment, will not compile aimrt_cli!")
    message(WARNING "try to install pyinstaller, jinja2 and pyyaml by `pip3 install pyinstaller jinja2 pyyaml --upgrade`")
  endif()
endif()

if(AIMRT_BUILD_WITH_PROTOBUF)
  include(GetProtobuf)
  include(ProtobufGenCode)

  if(AIMRT_USE_LOCAL_PROTOC_COMPILER)
    set(Protobuf_PROTOC_EXECUTABLE
        "protoc"
        CACHE STRING "Path to protoc compiler.")
    add_executable(aimrt::protoc IMPORTED GLOBAL)
    set_target_properties(aimrt::protoc PROPERTIES IMPORTED_LOCATION ${Protobuf_PROTOC_EXECUTABLE})
    set_property(GLOBAL PROPERTY PROTOC_NAMESPACE_PROPERTY "aimrt")
  endif()

  if(AIMRT_USE_PROTOC_PYTHON_PLUGIN)
    find_package(Python3 REQUIRED)
  endif()
endif()

if(AIMRT_BUILD_WITH_ROS2)
  # fix cmake policy for using FindPythonInterp and FindPythonLibs (ros2)
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
  endif()

  # fix gflag
  set(BUILD_TESTING
      OFF
      CACHE BOOL "")
  include(GetJsonCpp)
  find_package(rclcpp REQUIRED)
  find_package(Python3 REQUIRED)
endif()

if(AIMRT_BUILD_RUNTIME)
  include(GetBoost)
  include(GetGFlags)
  include(GetYamlCpp)
  include(GetTBB)

  if(AIMRT_BUILD_PYTHON_RUNTIME)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
    set(PYBIND11_PYTHON_VERSION ${Python3_VERSION})
    include(GetPybind11)

    if(AIMRT_BUILD_PYTHON_PACKAGE)
      include(CheckPythonPackage)
      check_python_package(build BUILD_FOUND)
      check_python_package(setuptools SETUPTOOLS_FOUND)
      check_python_package(wheel WHEEL_FOUND)

      if(NOT BUILD_FOUND
         OR NOT SETUPTOOLS_FOUND
         OR NOT WHEEL_FOUND)
        set(AIMRT_BUILD_PYTHON_PACKAGE OFF)
        message(WARNING "Can not find build, setuptools or wheel in your python environment, will not generate aimrt_py package! ")
        message(WARNING "try to install build, setuptools and wheel by `pip3 install build setuptools wheel --upgrade`")
      endif()
    endif()
  endif()

  if(AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN)
    include(GetSqlite)
  endif()

  if(AIMRT_BUILD_ICEORYX_PLUGIN)
    include(GetCppToml)
    include(GetIceoryx)
  endif()

  if(AIMRT_BUILD_MQTT_PLUGIN)
    include(GetPahoMqttC)
  endif()

  if(AIMRT_BUILD_ZENOH_PLUGIN)
    # find Rust compiler
    execute_process(
      COMMAND rustc --version
      RESULT_VARIABLE rustc_result
      OUTPUT_VARIABLE rustc_output
      ERROR_QUIET)

    if(rustc_result EQUAL 0)
      message(STATUS "Rust compiler (rustc) found: ${rustc_output}")
      include(GetZenoh)
    else()
      message(WARNING "Rust compiler (rustc) not found, will not compile zenoh plugin. Please install Rust environment referring to https://www.rust-lang.org/tools/install.")
      set(AIMRT_BUILD_ZENOH_PLUGIN OFF)
    endif()
  endif()

  if(AIMRT_BUILD_OPENTELEMETRY_PLUGIN)
    include(GetProtobuf)
    include(GetNlohmannJson)
    include(GetOpenTelemetryProto)
    include(GetOpenTelemetryCpp)
  endif()

  if(AIMRT_BUILD_GRPC_PLUGIN)
    include(GetNghttp2)
  endif()
endif()

# Add subdirectory
add_subdirectory(src)

if(AIMRT_INSTALL)
  # Install
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
    DESTINATION ./
    FILES_MATCHING
    PATTERN "*.cmake")

  install(EXPORT ${INSTALL_CONFIG_NAME} DESTINATION lib/cmake/${PROJECT_NAME})
endif()

# Put at last to ensure ros2 installation is done
if(AIMRT_INSTALL
   AND AIMRT_BUILD_PYTHON_RUNTIME
   AND AIMRT_BUILD_PYTHON_PACKAGE)
  install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" --build \"${CMAKE_BINARY_DIR}\" --config ${CMAKE_BUILD_TYPE} --target create_python_pkg)")
endif()
