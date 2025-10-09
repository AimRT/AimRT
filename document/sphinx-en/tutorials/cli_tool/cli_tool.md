# CLI Tool


## Introduction

**aimrt_cli** is an official command-line tool provided by AimRT, currently supporting the following features:

- [Generate scaffolding code for new projects](./gen_prj.md)

You can also directly run `aimrt_cli --help` to get the built-in help instructions. More features are coming soon.


## Installation
The **aimrt_cli** tool is a small utility developed in Python. There are three installation methods; please choose any one you prefer.


### Install from source into Python environment
**aimrt_cli** provides the ability to be packaged directly into the Python environment via the `setuptools` tool. After downloading the AimRT source code, execute the following commands in your terminal:

```
cd <path_to_your_aimrt_src_code>/src/tools/aimrt_cli
python -m build --wheel
pip install dist/aimrt_cli-*.whl
```

The above process will install an aimrt_cli executable into your Python environment, typically located in the `~/.local/bin/` directory (it may differ if you are using a virtual environment). This directory needs to be added to your `PATH` environment variable, after which you can run the `aimrt_cli` command.

You can uninstall it using `pip uninstall aimrt_cli`.


### Build executable from source
You can directly compile **aimrt** to generate the `aimrt_cli` executable and add it to the system's environment variables, allowing you to run the `aimrt_cli` command in the terminal. The overall process is:
- Execute the `build.sh` file in your aimrt source repository to compile. You can disable other CMake options you don't need in the `build.sh` file to speed up compilation.
- Find the compiled **aimrt_cli** executable in the build folder.
- Add it to the environment variables.

Refer to the following commands:

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