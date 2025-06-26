
# 安装与引用（CPP）

> **💡 推荐：快速开始**
> 如果您希望快速开始 AimRT 开发，强烈推荐使用 [开发容器](devcontainer.md)，它提供了预配置的完整开发环境，无需手动安装任何依赖。

## 系统要求

当前 AimRT 官方支持的操作系统和编译套件包括：
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

除以上列出的系统外，理论上所有兼容以上环境的系统均可以编译运行 AimRT。

Ubuntu 22.04 为推荐系统，所有测试以及二进制包均基于此系统，Windows 平台下仅支持 MSVC 编译器且插件支持较为有限。

ROS2 相关内容目前仅支持 humble 版本。

## 安装方式

当前 AimRT 的 C++ 接口仅支持从源码构建安装。

### 源码构建

[Ubuntu Linux 22.04 源码构建](build_from_source_ubuntu.md)

[Windows 源码构建](build_from_source_windows.md)

## 引用方式

在开发 C++ 工程时，您可以通过两种方式引用 AimRT：
- [推荐] 基于 CMake FetchContent，通过源码进行引用；
- 安装后，基于 CMake find_package 进行引用；

AimRT 比较轻量，推荐用户直接基于源码进行引用。

### 通过源码引用

您可以参考以下 CMake 代码引用 AimRT，注意需要将`GIT_TAG`版本改为你想引用的版本：
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

### 安装后find_package引用

参考 [源码构建](build_from_source_ubuntu.md) 运行 build.sh 脚本进行构建，在构建时可以修改 `CMAKE_INSTALL_PREFIX` 指定安装目录，完成安装后，参考以下步骤完成引用：
- 如果没有安装在系统路径，则需要在自己项目的 CMake 中设置 CMAKE_PREFIX_PATH 到 AimRT 的安装目录，例如：
  ```cmake
  list(APPEND CMAKE_PREFIX_PATH "/path/to/aimrt/install")
  ```
- 需要在自己项目的 CMake 中找到必须的依赖，可以直接使用 AimRT 安装后附带的 .cmake 脚本，参考以下代码：
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
- 如果编译 AimRT 时带上了 ROS 相关功能，还需要引用 AimRT 安装时引入的一些 ROS 包，例如：
  ```cmake
  find_package(ros2_plugin_proto REQUIRED)
  ```
- 最后，通过find_package来找到aimrt：
  ```cmake
  find_package(aimrt REQUIRED)
  ```


完整的示例 CMake 代码：
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
