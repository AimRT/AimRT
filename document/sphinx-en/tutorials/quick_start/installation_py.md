

# References & Installation (Python)

The AimRT Python interface is provided through the `aimrt_py` package. You can install the `aimrt_py` package via three methods:

- **[Not Supported Yet]** PyPI installation;
- Binary installation;
- Source code compilation and installation;

## Python Environment Requirements

The minimum Python version officially tested by AimRT is 3.10, with the minimum glibc version for Linux systems being 2.28 (check using `ldd --version` command).

We have tested the `aimrt_py` package on the following systems and Python versions:

- Ubuntu 22.04
  - Python 3.10
- Windows 10
  - Python 3.11

AimRT-Python supports all AimRT plugins and Protobuf/ROS2 message types. Its configuration files are almost identical to AimRT-Cpp except that pkg-related configurations are not required in Python, and the rest of configurations can be reused. The interface functionality is also largely consistent with AimRT-Cpp, but the RPC interface only provides synchronous mode without async/coroutine support.

## PyPI Installation

***TODO***

<!-- You can directly install via `pip install aimrt_py`. -->

## Binary Installation

You can find the aimrt_py wheel files on [AimRT's release page](https://github.com/AimRT/AimRT/releases) and install via pip.

## Source Compilation Installation

First download the source code via git or other methods, then refer to [Ubuntu Source Build](build_from_source_ubuntu.md)/[Windows Source Build](build_from_source_windows.md) for compilation. After building, the aimrt_py wheel file will be located in build/aimrt_py_pkg/dist path, then install via pip.

## Plugin Installation Instructions

AimRT adopts a plugin architecture where different plugins correspond to different functionalities. Plugins are dynamically loaded at runtime, essentially being dynamic library files. Some plugins may require separate installation due to dependency considerations.

After installation, check installed plugins using:

```bash
ls -l $(pip show aimrt_py | grep Location | awk '{print $2 "/aimrt_py"}')
```

This command will display all files under the installation path, where files with suffix ending with "plugin" are plugin files (linux: `*_plugin.so`, windows: `*_plugin.dll`).

<!-- PyPI installation doesn't include mqtt/ros2 plugins. Use source compilation or binary installation if needed. -->

Windows platform currently doesn't support ros2/mqtt/opentelemetry plugins.