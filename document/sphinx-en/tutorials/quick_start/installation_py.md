# Reference and Installation (Python)

> **💡 Recommendation: Quick Start**
> If you want to get started with AimRT development quickly, we strongly recommend using the [Development Container](devcontainer.md), which provides a pre-configured complete development environment.

The AimRT Python interface is used through the `aimrt_py` package. You can obtain the `aimrt_py` package in three ways:

- 【Not yet supported】PyPI installation;
- Binary installation;
- Source compilation installation;

## Python Environment Requirements

The lowest Python version officially tested by AimRT is 3.10, and the minimum glibc version on Linux systems is 2.28 (you can check this with the `ldd --version` command).

We have tested the `aimrt_py` package on the following systems and Python versions:

- Ubuntu 22.04
  - python 3.10
- Windows 10
  - python 3.11

AimRT-Python also supports all AimRT plugins and Protobuf, ROS2 message types. Its configuration files are almost identical to AimRT-Cpp, except that pkg-related configurations are not needed in Python, and the rest of the configuration files can be reused; its interface functionality is also basically identical to AimRT-Cpp, but the RPC interface only has a synchronous interface, with no asynchronous/coroutine RPC interfaces.

## PyPI Installation

***TODO***

<!-- You can install directly via `pip install aimrt_py`. -->

## Binary Installation

You can find the aimrt_py whl file directly on [AimRT's release page](https://github.com/AimRT/AimRT/releases) and install it via pip.

## Source Compilation Installation

First, download the source code via git or other methods, then refer to [Ubuntu Source Build](build_from_source_ubuntu.md)/ [Windows Source Build](build_from_source_windows.md) for building and compilation. After the build is complete, the aimrt_py whl file will be located in the build/aimrt_py_pkg/dist path, and can then be installed via pip.

## Plugin Installation Instructions

AimRT adopts a plugin-based design, where different plugins correspond to different functionalities. Plugins are dynamically loaded at runtime and are essentially dynamic library files. Due to some dependency-related reasons, certain plugins may need to be installed separately.

After installation, you can view the installed plugins as follows:


```bash
ls -l $(pip show aimrt_py | grep Location | awk '{print $2 "/aimrt_py"}')
```


This command will display all files in the installation path, where files with plugin as the filename suffix are plugin files (on Linux they are `*_plugin.so`, on Windows they are `*_plugin.dll`).

<!-- The PyPI installation method does not include plugins like mqtt, ros2, etc. If you need to use these plugins, you can install them via source compilation or by downloading binaries. -->

The Windows platform does not currently support plugins like ros2, mqtt, opentelemetry, etc.