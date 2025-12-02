# Ubuntu 22.04 Source Build

## Required Dependencies

### CMake

The minimum CMake version required for AimRT compilation is 3.24. The CMake version installed via apt package manager in Ubuntu 22.04 is 3.22, which does not meet the requirement. Please visit the [CMake official website](https://cmake.org/download/) to download the corresponding version for installation.

Two recommended installation methods:

1. Install CMake via pip

```bash
sudo apt install python3 python3-pip
pip install cmake --upgrade
```

2. Install the official CMake installation script

```bash
wget https://github.com/Kitware/CMake/releases/download/v3.30.4/cmake-3.30.4-linux-x86_64.sh
sudo bash cmake-3.30.4-linux-x86_64.sh --prefix=/usr/local --skip-license
```

### make/gcc/g++

The gcc version in Ubuntu 22.04 apt sources is 11.4, which is suitable for AimRT builds and can be installed directly

```bash
sudo apt install make gcc g++
```

## Minimal Build

After installing the above content, you can perform a minimal build without external dependencies. The build options are as follows

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
    -DAIMRT_BUILD_SERVICE_INTROSPECTION_PLUGIN=ON \
    -DAIMRT_BUILD_TOPIC_LOGGER_PLUGIN=ON \
    -DAIMRT_BUILD_OPENTELEMETRY_PLUGIN=ON \
    -DAIMRT_BUILD_GRPC_PLUGIN=ON \
    -DAIMRT_BUILD_PYTHON_PACKAGE=OFF

cmake --build build --config Release --target all -j
```

If you want to enable other options, you need to install the corresponding dependencies. For specific dependencies, please refer to [Optional Dependencies](#optional-dependencies)

## Optional Dependencies

### Python Features and Related Packages

The minimum Python version used by AimRT is 3.10, with version 3.10 recommended (the version installed from Ubuntu 22.04 apt sources).

AimRT's Python interface depends on Python 3.

```bash
sudo apt install python3
```

The aimrt_cli tool depends on Python 3 and the pyinstaller jinja2 pyyaml libraries, which can be installed with the following command

```bash
sudo apt install python3 python3-pip
pip install pyinstaller jinja2 pyyaml --upgrade
```

The function to package and generate the aimrt_py whl package depends on Python 3 and libraries such as build setuptools wheel, which can be installed with the following command

```bash
sudo apt install python3 python3-pip
pip install build setuptools wheel --upgrade
```

The above content corresponds to the following options

```bash
-DAIMRT_BUILD_PYTHON_RUNTIME=ON
-DAIMRT_BUILD_CLI_TOOLS=ON
-DAIMRT_BUILD_PYTHON_PACKAGE=ON
```

### ROS2 Related Dependencies

AimRT's ROS2 related features and ROS2 plugins depend on the ROS2 humble version (only this version is supported). You can refer to the [ROS2 official documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for installation.

After installation, you need to set the corresponding environment variables, namely

```bash
source /opt/ros/humble/setup.bash
```

You can check whether the corresponding environment variables are set with the following command

```bash
printenv | grep -i ROS
```

ROS2 related dependencies correspond to the following options

```bash
-DAIMRT_BUILD_WITH_ROS2=ON
-DAIMRT_BUILD_ROS2_PLUGIN=ON
```

### Iceoryx Related Dependencies

AimRT's Iceoryx plugin depends on the acl library, which can be installed with the following command

```bash
sudo apt install libacl1-dev
```

Iceoryx related dependencies correspond to the following options

```bash
-DAIMRT_BUILD_ICEORYX_PLUGIN=ON
```

### Zenoh Related Dependencies

AimRT's Zenoh plugin depends on the local rust environment, which can be installed with the following command

```bash
sudo apt install curl
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Zenoh related dependencies correspond to the following options

```bash
-DAIMRT_BUILD_ZENOH_PLUGIN=ON
```

### MQTT Related Dependencies

AimRT's MQTT plugin depends on the openssl library, which can be installed with the following command

```bash
sudo apt install libssl-dev
```

MQTT related dependencies correspond to the following options

```bash
-DAIMRT_BUILD_MQTT_PLUGIN=ON
```

## Full Build

After installing all the above content, you can perform a full build. You can directly run the build.sh script in the root directory for building

```bash
./build.sh
```

During the build process, AimRT will use FetchContent to pull dependencies from github. If the network environment is poor, you can use the following command to replace the FetchContent source to pull from the gitee mirror before building

```bash
source url_cn.bashrc
./build.sh $AIMRT_DOWNLOAD_FLAGS
```
