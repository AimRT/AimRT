# Installation & Usage (CPP)

> **💡 Recommendation: Quick Start**
> If you want to get started with AimRT development quickly, we strongly recommend using the [Development Container](devcontainer.md), which provides a pre-configured complete development environment without the need to manually install any dependencies.

## System Requirements

Currently, AimRT officially supports the following operating systems and compiler suites:
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

In addition to the systems listed above, theoretically all systems compatible with the above environments can compile and run AimRT.

Ubuntu 22.04 is the recommended system; all tests and binary packages are based on this system. On the Windows platform, only the MSVC compiler is supported and plugin support is relatively limited.

ROS2-related content currently only supports the humble version.

## Installation Methods

Currently, AimRT's C++ interface only supports installation from source.

### Build from Source

[Ubuntu Linux 22.04 Build from Source](build_from_source_ubuntu.md)

[Windows Build from Source](build_from_source_windows.md)

## Usage Methods

When developing a C++ project, you can use AimRT in two ways:
- [Recommended] Based on CMake FetchContent, referencing via source code;
- After installation, based on CMake find_package for referencing;

AimRT is relatively lightweight, and we recommend users directly reference it based on source code.

### Referencing via Source Code

You can reference AimRT with the following CMake code. Note that you need to change the `GIT_TAG` version to the version you want to reference:

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


### Referencing via find_package After Installation

Refer to [Build from Source](build_from_source_ubuntu.md) to run the build.sh script for building. During the build, you can modify `CMAKE_INSTALL_PREFIX` to specify the installation directory. After completing the installation, follow the steps below to complete the reference:
- If it is not installed in the system path, you need to set CMAKE_PREFIX_PATH in your project's CMake to the AimRT installation directory, for example:
  
```cmake
  list(APPEND CMAKE_PREFIX_PATH "/path/to/aimrt/install")
  ```

- You need to find the required dependencies in your project's CMake. You can directly use the .cmake scripts provided with the AimRT installation. Refer to the following code:
  
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

- If ROS-related features were included when compiling AimRT, you also need to reference some ROS packages introduced during AimRT installation, for example:
  
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
