# CLI Tool

## Introduction

**aimrt_cli** is an official command-line tool provided by AimRT, currently supporting the following features:

- [Generate scaffold code for new projects](./gen_prj.md)

You can also directly execute `aimrt_cli --help` to get built-in help instructions. More features are coming soon.

## Installation
**aimrt_cli** is a small Python-based tool with three installation methods. Please choose any one you prefer.

### Install from source into Python environment
**aimrt_cli** provides the functionality to package directly into the Python environment using `setuptools`. After downloading the AimRT source code, execute the following command in the terminal:
```
cd <path_to_your_aimrt_src_code>/src/tools/aimrt_cli
python -m build --wheel
pip install dist/aimrt_cli-*.whl
```
This process will install an aimrt_cli executable into your Python environment, typically located in the `~/.local/bin/` directory (may differ if using a virtual environment). This directory needs to be added to your `PATH` environment variable. You can then execute it using the `aimrt_cli` command.

Use `pip uninstall aimrt_cli` to uninstall.

### Compile executable from source
You can directly compile **aimrt** to generate the `aimrt_cli` executable file and add it to your system's environment variables, allowing you to execute the `aimrt_cli` command in the terminal. The overall process is:
- Execute the `build.sh` file in your AimRT source repository for compilation. You can disable other CMake options you don't need in the `build.sh` file to speed up compilation.
- The compiled **aimrt_cli** executable can be found in the build folder.
- Add it to your environment variables.

Reference commands:
```
cd <path_to_your_aimrt_src_code>
./build.sh

mv <path_to_your_aimrt_src_code>/build/install/bin/aimrt_cli <path_to_your_aimrt_cli>

export PATH=<path_to_your_aimrt_cli>:$PATH
```

### Direct pip installation

***TODO***

### Download executable from Release

***TODO***