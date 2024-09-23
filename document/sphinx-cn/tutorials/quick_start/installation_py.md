# 引用与安装（Python）

AimRT Python 接口通过 `aimrt_py` 包来使用。您可以通过三种方式安装获取 `aimrt_py` 包：

- 基于 pip install 安装；
- 二进制安装；
- [推荐] 基于源码编译安装；

## Python 环境要求

AimRT官方测试过的最低 Python 版本是 3.10，Linux 系统 glibc 的最低版本为 2.28（可以使用 `ldd --version` 命令查看）。

我们在以下系统和 python 版本上测试过 `aimrt_py` 包：

- Ubuntu22.04
  - python3.10
- Windows11
  - python3.11


请注意，如果您想要使用 AimRT-Python 中的 RPC 或 Channel 功能，当前只支持以 protobuf 作为协议，在使用时需要在本地安装有 protobuf python 包，您可以通过 `pip install protobuf` 来安装。

## PyPI 安装

***TODO***

<!-- 您可以直接通过 `pip install aimrt_py` 来安装。 -->

## 二进制安装

***TODO***

<!-- 您可以直接在 `AimRT 的发布页面` 上下载一些主流平台上编译好的二进制包，并在其中找到 aimrt_py 的 whl 文件，通过 pip 安装。 -->

## 源码编译安装

首先通过 git 等方式下载源码，然后基于 CMake 进行构建编译，构建完成后在 build/aimrt_py_package/dist 路径下有 aimrt_py 的 whl 文件，最后通过 pip 安装。

以下是在 Linux 平台（以 Ubuntu 22.04 为例）和 Windows 平台（使用 MSVC 工具链）上的编译安装方法。

### Linux 平台（以 Ubuntu 22.04 为例）

1. 下载源码
2. 安装编译依赖

   全量编译本项目需要安装 ros2（humble 版本，安装指引见 [ros2 installation](https://docs.ros.org/en/humble/Installation.html)），如果您不需要使用 ros2 插件可以在 build.sh 文件中将以下选项更改为 OFF：

   ```bash
   -DAIMRT_BUILD_WITH_ROS2=ON \
   -DAIMRT_BUILD_ROS2_PLUGIN=ON \
   # ...
   ```

   为支持打包为 .whl 文件，需要安装 python 模块 build、 setuptools 和 wheel，其中 setuptools 最低要求版本为 61.0，推荐直接安装最新版本，可以使用如下命令安装

    ```bash
    pip install build setuptools wheel --upgrade
    ```

3. 安装编译工具链

    为编译项目还需要安装 CMake 和 make，可以使用如下命令安装

    ```bash
    sudo apt install cmake make
    ```

    CMake 的最低版本要求为 3.24，如果系统中的版本低于 3.24，可以到 [CMake 官网](https://cmake.org/download/) 下载最新版本的 CMake 并安装。

    编译项目还需要使用编译器，推荐使用 GCC 11.4 或更高版本，在 Ubuntu 22.04 上可以使用如下命令安装

    ```bash
    sudo apt install gcc g++
    ```

    如果您的 gcc 版本低于 11.4，可以到 [GCC 官网](https://gcc.gnu.org/) 下载最新版本的 GCC 并安装。

4. 执行 build.sh 脚本编译生成 .whl 文件

    进入 aimrt 项目文件夹，执行 build.sh 脚本，命令如下

    ```bash
    ./build.sh
    ```

    编译完成后可以在 `build/aimrt_py_pkg/dist` 目录下找到 `aimrt_py*.whl` 文件，使用不同的 python 环境、系统架构和仓库版本文件名会略有不同，例如使用 CPython3.10 在 x86_64 架构下编译 0.7.0 版本仓库生成的文件名为 `aimrt_py-0.7.0-cp310-cp310-linux_x86_64.whl`。

5. 使用 pip 安装 .whl 文件

    使用 pip 安装编译生成的 .whl 文件，命令如下

    ```bash
    pip install build/aimrt_py_pkg/dist/aimrt_py*.whl
    ```

    安装完成后即可使用 aimrt_py 包。

### Windows 平台 （使用 MSVC 工具链）

1. 下载源码
2. 安装编译依赖

    Windows 平台不支持 ros2、mqtt、opentelemetry 插件，build.bat 文件中默认关闭了这些插件。

   为支持打包为 .whl 文件，需要安装 python 模块 build 和 setuptools，可以使用如下命令安装

    ```shell
    pip install build setuptools
    ```

3. 安装编译工具链

    为编译项目还需要安装 Visual Studio 2019 或 Visual Studio 2022（推荐），可以到 [Visual Studio 官网](https://visualstudio.microsoft.com/zh-hans/vs/) 下载并安装。

4. 执行 build.bat 脚本编译生成 .whl 文件

    在 Command Prompt 中，进入 aimrt 项目文件夹，执行 build.bat 脚本，命令如下：

    ```shell
    ./build.bat
    ```

    编译完成后可以在 `build\aimrt_py_pkg\dist` 目录下找到 `aimrt_py*.whl` 文件，使用不同的 python 环境、系统架构和仓库版本文件名会略有不同，例如使用 CPython3.10 在 x86_64 架构下编译 0.7.0 版本仓库生成的文件名为 `aimrt_py-0.7.0-cp310-cp310-win_amd64.whl`。

5. 使用 pip 安装 .whl 文件

    使用 pip 安装编译生成的 .whl 文件，命令如下

    ```shell
    pip install build\aimrt_py_pkg\dist\aimrt_py*.whl
    ```

    安装完成后即可使用 aimrt_py 包。

## 插件安装说明

AimRT 采用插件化的设计，不同的插件对应不同的功能，插件采用运行时动态加载，本质就是一个动态库文件，由于一些依赖相关的原因，某些插件可能需要单独安装。

安装后可以通过如下方式查看已安装的插件：

```bash
ls -l $(pip show aimrt_py | grep Location | awk '{print $2 "/aimrt_py"}')
```

该命令会显示安装路径下的所有文件，其中文件名后缀以 plugin 结尾的文件即为插件文件（linux 下为 `*_plugin.so`, windows 下为 `*_plugin.dll`）。

PyPI 安装方式中不含 mqtt、ros2 等插件，如果需要使用这些插件，可以通过源码编译或者下载二进制的方式安装。

Windows 平台暂不支持 ros2、mqtt、opentelemetry 等插件。
