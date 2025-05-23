# References and Installation (Python)

The AimRT Python interface is accessed through the `aimrt_py` package. You can obtain and install the `aimrt_py` package in three ways:

- [Not yet supported] PyPI installation;
- Binary installation;
- Source code compilation and installation;

## Python Environment Requirements

The minimum Python version officially tested by AimRT is 3.10, and the minimum glibc version for Linux systems is 2.28 (you can check this using the `ldd --version` command).

We have tested the `aimrt_py` package on the following systems and Python versions:

- Ubuntu 22.04
  - Python 3.10
- Windows 10
  - Python 3.11

AimRT-Python also supports all AimRT plugins and Protobuf/ROS2 message types. Its configuration files are almost entirely consistent with AimRT-Cpp, except that pkg-related configurations are not required in Python—all other parts of the configuration can be reused. The interface functionality is also largely identical to AimRT-Cpp, but the RPC interface only has a synchronous version, with no asynchronous/coroutine RPC interfaces.

## PyPI Installation

***TODO***

<!-- You can directly install it via `pip install aimrt_py`. -->

## Binary Installation

You can directly find the whl files for aimrt_py on the [AimRT releases page](https://github.com/AimRT/AimRT/releases) and install them using pip.

## Source Code Compilation and Installation

First, download the source code via git or other methods, then refer to [Ubuntu Source Build](build_from_source_ubuntu.md) or [Windows Source Build](build_from_source_windows.md) for compilation. After building, the whl files for aimrt_py will be located in the `build/aimrt_py_pkg/dist` directory. Finally, install them using pip.

## Plugin Installation Instructions

AimRT adopts a plugin-based design, where different plugins correspond to different functionalities. Plugins are dynamically loaded at runtime and are essentially dynamic library files. Due to dependency-related reasons, some plugins may require separate installation.

After installation, you can view the installed plugins using the following method:

```bash
ls -l $(pip show aimrt_py | grep Location | awk '{print $2 "/aimrt_py"}')
```

This command will display all files in the installation path. Files with names ending in `plugin` are plugin files (on Linux, `*_plugin.so`; on Windows, `*_plugin.dll`).

<!-- The PyPI installation method does not include plugins like mqtt and ros2. If you need these plugins, you can install them via source code compilation or by downloading binaries. -->

Windows platform currently does not support plugins such as ros2, mqtt, and opentelemetry.