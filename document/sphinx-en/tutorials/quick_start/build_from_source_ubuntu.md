

# Ubuntu 22.04 Source Code Build

## Required Dependencies

### CMake

The minimum CMake version required for AimRT compilation is 3.24. The CMake version 3.22 provided by Ubuntu 22.04's apt package manager does not meet the requirement. Please download the corresponding version from the [CMake Official Website](https://cmake.org/download/) for installation.

Two recommended installation methods:

1. Install CMake via pip

```bash
sudo apt install python3 python3-pip
pip install cmake --upgrade
```

2. Use official CMake installation script

```bash
wget https://github.com/Kitware/CMake/releases/download/v3.30.4/cmake-3.30.4-linux-x86_64.sh
sudo bash cmake-3.30.4-linux-x86_64.sh --prefix=/usr/local --skip-license
```

### make/gcc/g++

The gcc version 11.4 provided by Ubuntu 22.04's apt repository is suitable for AimRT build and can be directly installed:

```bash
sudo apt install make gcc g++
```

## Minimal Build

After installing the above components, you can perform a minimal build without external dependencies. The build options are as follows:

```bash
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DAIMRT_INSTALL=ON \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    -DAIMRT_BUILD_TESTS=OFF \
    -DAIMRT_BUILD_EXAMPLES=ON \
    -DAIMRT_BUILD_PROTOCOLS=ON \
    -DAIMRT_BUILD_DOCUMENT=OFF \
    -DAIMRT_BUILD_RUNTIME=ON \
    -DAIMRT_BUILD_CLI_TOOLS=OFF \
    -DAIMRT_BUILD_PYTHON_RUNTIME=OFF \
    -DAIMRT_USE_FMT_LIB=ON \
    -DAIMRT_BUILD_WITH_PROTOBUF=ON \
    -DAIMRT_USE_LOCAL_PROTOC_COMPILER=OFF \
    -DAIMRT_USE_PROTOC_PYTHON_PLUGIN=OFF \
    -DAIMRT_BUILD_WITH_ROS2=OFF \
    -DAIMRT_BUILD_NET_PLUGIN=ON \
    -DAIMRT_BUILD_MQTT_PLUGIN=OFF \
    -DAIMRT_BUILD_ZENOH_PLUGIN=OFF \
    -DAIMRT_BUILD_ICEORYX_PLUGIN=OFF \
    -DAIMRT_BUILD_ROS2_PLUGIN=OFF \
    -DAIMRT_BUILD_RECORD_PLAYBACK_PLUGIN=ON \
    -DAIMRT_BUILD_TIME_MANIPULATOR_PLUGIN=ON \
    -DAIMRT_BUILD_PARAMETER_PLUGIN=ON \
    -DAIMRT_BUILD_LOG_CONTROL_PLUGIN=ON \
    -DAIMRT_BUILD_TOPIC_LOGGER_PLUGIN=ON \
    -DAIMRT_BUILD_OPENTELEMETRY_PLUGIN=ON \
    -DAIMRT_BUILD_GRPC_PLUGIN=ON \
    -DAIMRT_BUILD_PYTHON_PACKAGE=OFF

cmake --build build --config Release --target all -j
```

To enable other options, install corresponding dependencies as described in [Optional Dependencies](#optional-dependencies)

## Optional Dependencies

### Python Functionality and Related Packages

AimRT requires Python 3.10 or higher (recommend using Python 3.10 from Ubuntu 22.04's apt repository).

AimRT's Python interface depends on Python 3:

```bash
sudo apt install python3
```

The aimrt_cli tool requires Python 3 and three libraries: pyinstaller, jinja2, and pyyaml. Install using:

```bash
sudo apt install python3 python3-pip
pip install pyinstaller jinja2 pyyaml --upgrade
```

Building aimrt_py wheel package requires Python 3 and libraries including build, setuptools, and wheel. Install using:

```bash
sudo apt install python3 python3-pip
pip install build setuptools wheel --upgrade
```

These components correspond to the following build options:

```bash
-DAIMRT_BUILD_PYTHON_RUNTIME=ON
-DAIMRT_BUILD_CLI_TOOLS=ON
-DAIMRT_BUILD_PYTHON_PACKAGE=ON
```

### ROS2 Related Dependencies

AimRT's ROS2 functionality and plugins require ROS2 Humble version (only this version is supported). Refer to [ROS2 Official Documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for installation.

After installation, set environment variables:

```bash
source /opt/ros/humble/setup.bash
```

Verify environment variables with:

```bash
printenv | grep -i ROS
```

ROS2 dependencies correspond to:

```bash
-DAIMRT_BUILD_WITH_ROS2=ON
-DAIMRT_BUILD_ROS2_PLUGIN=ON
```

### Iceoryx Related Dependencies

AimRT's Iceoryx plugin requires acl library. Install using:

```bash
sudo apt install libacl1-dev
```

Iceoryx dependencies correspond to:

```bash
-DAIMRT_BUILD_ICEORYX_PLUGIN=ON
```

### Zenoh Related Dependencies

AimRT's Zenoh plugin requires local Rust environment. Install using:

```bash
sudo apt install curl
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Zenoh dependencies correspond to:

```bash
-DAIMRT_BUILD_ZENOH_PLUGIN=ON
```

### MQTT Related Dependencies

AimRT's MQTT plugin requires openssl library. Install using:

```bash
sudo apt install libssl-dev
```

MQTT dependencies correspond to:

```bash
-DAIMRT_BUILD_MQTT_PLUGIN=ON
```

## Full Build

After installing all dependencies, perform full build using the build.sh script in root directory:

```bash
./build.sh
```

For network-constrained environments, use gitee mirror for FetchContent dependencies:

```bash
source url_cn.bashrc
./build.sh $AIMRT_DOWNLOAD_FLAGS
```