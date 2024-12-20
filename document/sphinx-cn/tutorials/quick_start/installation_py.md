# 引用与安装（Python）

AimRT Python 接口通过 `aimrt_py` 包来使用。您可以通过三种方式安装获取 `aimrt_py` 包：

- 【暂不支持】PyPI 安装；
- 二进制安装；
- 源码编译安装；

## Python 环境要求

AimRT官方测试过的最低 Python 版本是 3.10，Linux 系统 glibc 的最低版本为 2.28（可以使用 `ldd --version` 命令查看）。

我们在以下系统和 python 版本上测试过 `aimrt_py` 包：

- Ubuntu 22.04
  - python 3.10
- Windows 10
  - python 3.11


请注意，如果您想要使用 AimRT-Python 中的 RPC 或 Channel 功能，当前只支持以 protobuf 作为协议，在使用时需要在本地安装有 protobuf python 包，您可以通过 `pip install protobuf` 来安装。

## PyPI 安装

***TODO***

<!-- 您可以直接通过 `pip install aimrt_py` 来安装。 -->

## 二进制安装

您可以直接在 [AimRT 的发布页面](https://github.com/AimRT/AimRT/releases) 中找到 aimrt_py 的 whl 文件，通过 pip 安装。

## 源码编译安装

首先通过 git 等方式下载源码，然后参考 [Ubuntu 源码构建](build_from_source_ubuntu.md)/ [Windows 源码构建](build_from_source_windows.md) 进行构建编译，构建完成后在 build/aimrt_py_pkg/dist 路径下有 aimrt_py 的 whl 文件，最后通过 pip 安装。

## 插件安装说明

AimRT 采用插件化的设计，不同的插件对应不同的功能，插件采用运行时动态加载，本质就是一个动态库文件，由于一些依赖相关的原因，某些插件可能需要单独安装。

安装后可以通过如下方式查看已安装的插件：

```bash
ls -l $(pip show aimrt_py | grep Location | awk '{print $2 "/aimrt_py"}')
```

该命令会显示安装路径下的所有文件，其中文件名后缀以 plugin 结尾的文件即为插件文件（linux 下为 `*_plugin.so`, windows 下为 `*_plugin.dll`）。

<!-- PyPI 安装方式中不含 mqtt、ros2 等插件，如果需要使用这些插件，可以通过源码编译或者下载二进制的方式安装。 -->

Windows 平台暂不支持 ros2、mqtt、opentelemetry 等插件。
