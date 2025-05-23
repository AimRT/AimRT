# Ubuntu 22.04 Source Build

## Required Dependencies

### CMake

The minimum CMake version required for AimRT compilation is 3.24. The CMake version installed via apt package manager in Ubuntu 22.04 is 3.22, which does not meet the requirement. Please visit the [CMake official website](https://cmake.org/download/) to download and install the corresponding version.

There are two recommended installation methods:

1. Install CMake via pip

```bash
sudo apt install python3 python3-pip
pip install cmake --upgrade
```

2. Install using the official CMake installation script

```bash
wget https://github.com/Kitware/CMake/releases/download/v3.30.4/cmake-3.30.4-linux-x86_64.sh
sudo bash cmake-3.30.4-linux-x86_64.sh --prefix=/usr/local --skip-license
```

### make/gcc/g++

The gcc version provided by apt in Ubuntu 22.04 is 11.4, which is suitable for AimRT build. It can be directly installed:

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

To enable other options, you need to install corresponding dependencies. Please refer to [Optional Dependencies](#optional-dependencies) for specific requirements.

## Optional Dependencies

### Python Functionality and Related Packages

The minimum Python version required by AimRT is 3.10. Version 3.10 (the version installed via apt in Ubuntu 22.04) is recommended.

AimRT's Python interface depends on Python 3.

```bash
sudo apt install python3
```

The aimrt_cli tool requires Python 3 and three libraries: pyinstaller, jinja2, and pyyaml. They can be installed with the following command:

```bash
sudo apt install python3 python3-pip
pip install pyinstaller jinja2 pyyaml --upgrade
```

The wheel package generation functionality for aimrt_py depends on Python 3 and libraries like build, setuptools, and wheel. They can be installed with:

```bash
sudo apt install python3 python3-pip
pip install build setuptools wheel --upgrade
```

The above components correspond to the following options:

```bash
-DAIMRT_BUILD_PYTHON_RUNTIME=ON
-DAIMRT_BUILD_CLI_TOOLS=ON
-DAIMRT_BUILD_PYTHON_PACKAGE=ON
```

### ROS2 Related Dependencies

AimRT's ROS2 functionality and ROS2 plugins depend on ROS2 Humble version (only this version is supported). Please refer to the [ROS2 official documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for installation.

After installation, you need to set the corresponding environment variables:

```bash
source /opt/ros/humble/setup.bash
```

You can verify the environment variables with:

```bash
printenv | grep -i ROS
```

ROS2 related dependencies correspond to the following options:

```bash
-DAIMRT_BUILD_WITH_ROS2=ON
-DAIMRT_BUILD_ROS2_PLUGIN=ON
```

### Iceoryx Related Dependencies

AimRT's Iceoryx plugin depends on the acl library, which can be installed with:

```bash
sudo apt install libacl1-dev
```

Iceoryx related dependencies correspond to the following options:

```bash
-DAIMRT_BUILD_ICEORYX_PLUGIN=ON
```

### Zenoh Related Dependencies

AimRT's Zenoh plugin requires a local Rust environment, which can be installed with:

```bash
sudo apt install curl
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Zenoh related dependencies correspond to the following options:

```bash
-DAIMRT_BUILD_ZENOH_PLUGIN=ON
```

### MQTT Related Dependencies

AimRT's MQTT plugin depends on the openssl library, which can be installed with:

```bash
sudo apt install libssl-dev
```

MQTT related dependencies correspond to the following options:

```bash
-DAIMRT_BUILD_MQTT_PLUGIN=ON
```

## Full Build

After installing all the above components, you can perform a full build. You can directly run the build.sh script in the root directory:

```bash
./build.sh
```

During the build process, AimRT will use FetchContent to pull dependencies from GitHub. If your network environment is poor, you can replace the FetchContent source with Gitee mirror before building:

```bash
source url_cn.bashrc
./build.sh $AIMRT_DOWNLOAD_FLAGS
```