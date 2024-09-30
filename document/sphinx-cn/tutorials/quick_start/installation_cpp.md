
# 引用与安装（CPP）


在开发 C++ 工程时，您可以通过两种方式引用 AimRT：
- [推荐] 基于 CMake FetchContent，通过源码进行引用；
- 安装后，基于 CMake find_package 进行引用；

AimRT 比较轻量，推荐用户直接基于源码进行引用。如果要使用基于安装的方式进行引用，AimRT 也提供了两种方式：
- 从源码编译安装；
- 二进制安装；


## 编译环境要求
AimRT 兼容 linux、windows 等主流操作系统，编译器需要能够支持 c++20，CMake 版本需要 3.24 或以上。我们已经在以下操作系统和编译器上测试过：
- Ubuntu22.04
  - gcc-11.4
  - gcc-12.4
  - gcc-13.3
  - clang-15.0.7
  - clang-16.0.6
  - clang-17.0.6
  - clang-18.1.8
- Windows11
  - MSVC-19.40

请注意：
- 在编译构建时，AimRT 可能通过源码方式引用一些第三方依赖，如果出现网络问题，可以参考[CMake](../concepts/cmake.md)文档进行处理。
- 在打开某些选项、编译某些插件时，AimRT 可能需要额外引用一些第三方依赖，细节请参考对应插件的文档、CMake 代码文件或构建时的提示。以下是一些示例：
  - 如果要编译 ROS2 相关接口/插件，AimRT 会通过`find_package`的方式在本地寻找 rclcpp 等依赖，请确保本地安装有[ROS2 Humble](https://docs.ros.org/en/humble/)。
  - 如果要构建 Python 接口、cli 工具等，AimRT 会通过`find_package`的方式在本地寻找 Python 依赖，请确保本地安装有 Python3 及相关的 python 包，缺失相应包在 CMake 生成过程中会告警不会编译相应内容。
  - Zenoh 插件需要本地安装有 rust 环境，在 Ubuntu 上可通过 `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh` 进行安装，没有 rust 环境则不会编译 zenoh 插件。
  - Iceoryx 插件依赖 libacl，在 ubuntu 上可通过 `sudo apt install libacl1-dev` 进行安装，选项开启但缺失相应包会报错。
  - Boost 依赖由于其内容较大，CMake 会首先检查本地是否有 1.82.0 及以上版本的 boost，如果有则使用本地安装的 boost，如果没有则进行下载。


## 通过源码引用

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

## 安装后find_package引用

请注意：使用 **install 后 find_package 引用**这种方式，只能引用部分功能，只能基于 pkg 模式开发，无法使用 app 模式开发、或开发 aimrt 插件。

### 安装方式一：从源码构建安装

基于源码编译 AimRT 非常简单，可参考以下步骤：
- 首先请确认本地环境符合要求；
- 通过 git 等方式下载源码；
- 直接执行 AimRT 源码根路径下的 build 脚本执行编译、安装；
  - linux 下请执行 build.sh;
  - windows 下请执行 build.bat;
  - 可以修改 build 脚本中的 CMake 选项以开关一些功能；
  - 可以修改 build 脚本中的`CMAKE_INSTALL_PREFIX`选项指定安装地址；
  - 如果遇到网络问题无法下载一些依赖，请参考[AimRT中的CMake](../concepts/cmake.md)文档中的说明；


### 安装方式二：从二进制包安装

***TODO***

<!-- 您可以直接在`AimRT 的发布页面`上下载一些主流平台上编译好的二进制包并安装。

注意：
- 部分插件只在一些平台上提供，这和插件本身所需的组件依赖的平台有关。
- AimRT 二进制安装包直接支持的平台较少，但并不意味 AimRT 仅支持这些平台。AimRT 本身比较轻量，没有太多依赖，鼓励使用源码形式安装/引用。 -->


### 安装完成后，使用CMake find_package进行引用

完成安装后，参考以下步骤完成引用：
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

find_package(ros2_plugin_proto REQUIRED)
find_package(aimrt REQUIRED)

# 引入后直接使用target_link_libraries链接aimrt的target
target_link_libraries(
  my_module
  PUBLIC aimrt::interface::aimrt_module_cpp_interface)
```
