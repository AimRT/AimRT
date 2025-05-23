# Installation and Reference (CPP)

## System Requirements

Currently, the officially supported operating systems and compiler suites for AimRT include:
- Ubuntu 22.04
  - gcc-11.4
  - gcc-12.4
  - gcc-13.3
  - clang-15.0.7
  - clang-16.0.6
  - clang-17.0.6
  - clang-18.1.8
- Windows
  - MSVC-1940

In addition to the systems listed above, theoretically all systems compatible with these environments can compile and run AimRT.

Ubuntu 22.04 is the recommended system, as all testing and binary packages are based on this system. On Windows platforms, only the MSVC compiler is supported, and plugin support is relatively limited.

ROS2-related content currently only supports the humble version.

## Installation Methods

Currently, the C++ interface of AimRT only supports installation through source code compilation.

### Source Code Compilation

[Ubuntu Linux 22.04 Source Code Compilation](build_from_source_ubuntu.md)

[Windows Source Code Compilation](build_from_source_windows.md)

## Reference Methods

When developing C++ projects, you can reference AimRT in two ways:
- [Recommended] Based on CMake FetchContent, referencing through source code;
- After installation, referencing based on CMake find_package;

AimRT is relatively lightweight, and it is recommended that users directly reference it through source code.

### Referencing via Source Code

You can refer to the following CMake code to reference AimRT. Note that you need to change the `GIT_TAG` version to the version you want to reference:
```cmake
include(FetchContent)

# set the version of AimRT to reference
FetchContent_Declare(
  aimrt
  GIT_REPOSITORY https://github.com/AimRT/aimrt.git
  GIT_TAG v1.x.x)

FetchContent_GetProperties(aimrt)

if(NOT aimrt_POPULATED)
  # Set some compilation options for AimRT
  set(AIMRT_BUILD_TESTS OFF CACHE BOOL "")
  set(AIMRT_BUILD_EXAMPLES OFF CACHE BOOL "")

  FetchContent_MakeAvailable(aimrt)
endif()

# After referencing, directly use target_link_libraries to link the target of AimRT
target_link_libraries(
  my_module
  PUBLIC aimrt::interface::aimrt_module_cpp_interface)
```

### Referencing via find_package After Installation

Refer to [Source Code Compilation](build_from_source_ubuntu.md) to run the build.sh script for compilation. During compilation, you can modify `CMAKE_INSTALL_PREFIX` to specify the installation directory. After completing the installation, follow these steps to complete the reference:
- If it is not installed in the system path, you need to set CMAKE_PREFIX_PATH in your project's CMake to the AimRT installation directory, for example:
  ```cmake
  list(APPEND CMAKE_PREFIX_PATH "/path/to/aimrt/install")
  ```
- You need to find the necessary dependencies in your project's CMake. You can directly use the .cmake script included with AimRT after installation. Refer to the following code:
  ```cmake
  list(APPEND CMAKE_MODULE_PATH /path/to/aimrt/install/cmake)
  include(GetFmt)
  include(GetLibUnifex)
  include(GetProtobuf)
  include(GetYamlCpp)
  include(GetJsonCpp)
  include(GetTBB)
  include(GetAsio)
  ```
- If ROS-related features are included when compiling AimRT, you also need to reference some ROS packages introduced during AimRT installation, for example:
  ```cmake
  find_package(ros2_plugin_proto REQUIRED)
  ```
- Finally, use find_package to locate aimrt:
  ```cmake
  find_package(aimrt REQUIRED)
  ```

Complete example CMake code:
```cmake
list(APPEND CMAKE_PREFIX_PATH "/path/to/aimrt/install")
list(APPEND CMAKE_MODULE_PATH "/path/to/aimrt/install/cmake")

include(GetFmt)
include(GetLibUnifex)
include(GetProtobuf)
include(GetYamlCpp)
include(GetJsonCpp)
include(GetTBB)
include(GetAsio)

find_package(ros2_plugin_proto REQUIRED)
find_package(aimrt REQUIRED)

# 引入后直接使用target_link_libraries链接aimrt的target
target_link_libraries(
  my_module
  PUBLIC aimrt::interface::aimrt_module_cpp_interface)
```