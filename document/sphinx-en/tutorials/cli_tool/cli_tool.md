

# CLI Tool

## Introduction

**aimrt_cli** is an official command-line tool provided by Agibot, currently supporting the following features:

- [Generate scaffold code for new projects](./gen_prj.md)

You can also directly execute `aimrt_cli --help` to view built-in help documentation. More features coming soon.

## Installation
**aimrt_cli** is a Python-based utility with three installation methods. Please choose whichever you prefer.

### Install from Source into Python Environment
**aimrt_cli** can be packaged into Python environments using `setuptools`. After downloading the AimRT source code, execute the following commands in terminal:
```
cd <path_to_your_aimrt_src_code>/src/tools/aimrt_cli
python -m build --wheel
pip install dist/aimrt_cli-*.whl
```
This will install an aimrt_cli executable to your Python environment (typically in `~/.local/bin/`, or virtual environment path). Ensure this directory is in your `PATH` environment variable, then use `aimrt_cli` command directly.

Uninstall with `pip uninstall aimrt_cli`.

### Compile Executable from Source
You can directly compile **aimrt** to generate the `aimrt_cli` executable. Follow these steps:
- Execute `build.sh` in your aimrt source repository (you may disable unnecessary CMake options to speed up compilation)
- Find the compiled **aimrt_cli** executable in the build folder
- Add it to your system PATH

Reference commands:
```
cd <path_to_your_aimrt_src_code>
./build.sh

mv <path_to_your_aimrt_src_code>/build/install/bin/aimrt_cli <path_to_your_aimrt_cli>

export PATH=<path_to_your_aimrt_cli>:$PATH
```

### Direct pip Installation

***TODO***

### Download Executable from Release

***TODO***