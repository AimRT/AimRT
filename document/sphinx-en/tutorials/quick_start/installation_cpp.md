

# Installation & Reference (CPP)

## System Requirements

Currently officially supported operating systems and compilation suites for AimRT include:
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

Theoretically all systems compatible with the above environments can compile and run AimRT besides those explicitly listed.

Ubuntu 22.04 is the recommended system, as all testing and binary packages are based on this OS. Windows platform only supports MSVC compiler with limited plugin support.

ROS2 related content currently only supports humble version.

## Installation Methods

The C++ interface of AimRT currently only supports source code compilation and installation.

### Source Code Compilation

[Ubuntu Linux 22.04 Source Build](build_from_source_ubuntu.md)

[Windows Source Build](build_from_source_windows.md)

## Reference Methods

When developing C++ projects, you can reference AimRT in two ways:
- [Recommended] Via CMake FetchContent using source code;
- After installation, using CMake find_package;

AimRT is relatively lightweight, we recommend users reference it directly through source code.

### Referencing via Source Code

You can reference AimRT using the following CMake code. Note to modify the `GIT_TAG` version to your desired release version:
```cmake
include(FetchContent)

# 可以指定aimrt地址和版本
FetchContent_Declare(
  aimrt
  GIT_REPOSITORY https://github.com/AimRT/aimrt.git
  GIT_TAG v1.x.x)

FetchContent_GetProperties(aimrt)

if(NOT aimrt_POPULATED)
  # 设置AimRT的一些编译选项
  set(AIMRT_BUILD_TESTS OFF CACHE BOOL "")
  set(AIMRT_BUILD_EXAMPLES OFF CACHE BOOL "")

  FetchContent_MakeAvailable(aimrt)
endif()

# 引入后直接使用target_link_libraries链接aimrt的target
target_link_libraries(
  my_module
  PUBLIC aimrt::interface::aimrt_module_cpp_interface)
```

### find_package Reference After Installation

Refer to [Source Build](build_from_source_ubuntu.md) to run build.sh script for compilation. During compilation, you can modify `CMAKE_INSTALL_PREFIX` to specify installation directory. After installation, follow these steps to reference:
- If not installed in system path, set CMAKE_PREFIX_PATH in your project's CMake to AimRT installation directory, e.g.:
  ```cmake
  list(APPEND CMAKE_PREFIX_PATH "/path/to/aimrt/install")
  ```
- Need to find required dependencies in your project's CMake. You can directly use the .cmake script included with AimRT installation. Refer to following code:
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
- If ROS-related features are included when compiling AimRT, you also need to reference some ROS packages introduced during AimRT installation, e.g.:
  ```cmake
  find_package(ros2_plugin_proto REQUIRED)
  ```
- Finally, find aimrt via find_package:
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