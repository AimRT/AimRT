# Ubuntu 22.04 源码构建

## 必选依赖

### CMake

AimRT 编译所需的最小 CMake 版本为 3.24，Ubuntu 22.04 使用 apt 包管理器安装的 CMake 版本为 3.22 不符合要求，请前往 [CMake 官方网站](https://cmake.org/download/) 下载对应版本进行安装。

推荐的安装方式有两种

1. 通过 pip 安装 CMake

```bash
sudo apt install python3 python3-pip
pip install cmake --upgrade
```

2. 安装 CMake 官方提供的安装脚本

```bash
wget https://github.com/Kitware/CMake/releases/download/v3.30.4/cmake-3.30.4-linux-x86_64.sh
sudo bash cmake-3.30.4-linux-x86_64.sh --prefix=/usr/local --skip-license
```

### make/gcc/g++

Ubuntu 22.04 中 apt 源自带的 gcc 版本为 11.4，适用于 AimRT 构建所用，可以直接安装

```bash
sudo apt install make gcc g++
```

## 最小化构建

将以上内容进行安装后即可进行无外部依赖的最小化构建，构建选项如下所示

```bash
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DAIMRT_INSTALL=ON \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    -DAIMRT_BUILD_TESTS=OFF \
    -DAIMRT_BUILD_EXAMPLES=ON \
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
    -DAIMRT_BUILD_OPENTELEMETRY_PLUGIN=ON \
    -DAIMRT_BUILD_GRPC_PLUGIN=ON \
    -DAIMRT_BUILD_PYTHON_PACKAGE=OFF

cmake --build build --config Release --target all -j
```

如果要开启其他选项则需要安装对应依赖，具体依赖请参考 [可选依赖](#可选依赖)

## 可选依赖

### Python 功能及其相关包

AimRT 所使用的 Python 版本最低为 3.10，推荐使用 3.10 版本（Ubuntu 22.04 的 apt 源安装版本）。

AimRT 的 Python 接口依赖 Python 3。

```bash
sudo apt install python3
```

aimrt_cli 工具依赖 Python 3 以及 pyinstaller jinja2 pyyaml 三个库，可以通过以下命令进行安装

```bash
sudo apt install python3 python3-pip
pip install pyinstaller jinja2 pyyaml --upgrade
```

打包生成 aimrt_py 的 whl 包功能依赖 Python 3 以及 build setuptools wheel 等库，可以通过以下命令进行安装

```bash
sudo apt install python3 python3-pip
pip install build setuptools wheel --upgrade
```

以上内容分别对应以下选项

```bash
-DAIMRT_BUILD_PYTHON_RUNTIME=ON
-DAIMRT_BUILD_CLI_TOOLS=ON
-DAIMRT_BUILD_PYTHON_PACKAGE=ON
```

### ROS2 相关依赖

AimRT 的 ROS2 相关功能以及 ROS2 插件依赖 ROS2 humble 版本（仅支持该版本），可以参考 [ROS2 官方文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) 进行安装。

安装完成后需要设置好相应的环境变量，即

```bash
source /opt/ros/humble/setup.bash
```

可以通过以下命令检查是否设置好相应的环境变量

```bash
printenv | grep -i ROS
```

ROS2 相关依赖对应以下选项

```bash
-DAIMRT_BUILD_WITH_ROS2=ON
-DAIMRT_BUILD_ROS2_PLUGIN=ON
```

### Iceoryx 相关依赖

AimRT 的 Iceoryx 插件依赖 acl 库，可以通过以下命令进行安装

```bash
sudo apt install libacl1-dev
```

Iceoryx 相关依赖对应以下选项

```bash
-DAIMRT_BUILD_ICEORYX_PLUGIN=ON
```

### Zenoh 相关依赖

AimRT 的 Zenoh 插件依赖本地的 rust 环境，可以通过以下命令进行安装

```bash
sudo apt install curl
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Zenoh 相关依赖对应以下选项

```bash
-DAIMRT_BUILD_ZENOH_PLUGIN=ON
```

### MQTT 相关依赖

AimRT 的 MQTT 插件依赖 openssl 库，可以通过以下命令进行安装

```bash
sudo apt install libssl-dev
```

MQTT 相关依赖对应以下选项

```bash
-DAIMRT_BUILD_MQTT_PLUGIN=ON
```

## 完整构建

将以上内容进行安装后即可进行完整构建，可以直接运行根目录下的 build.sh 脚本进行构建

```bash
./build.sh
```

AimRT 在构建过程中会使用 FetchContent 从 github 上拉取依赖，如果网络环境不佳，可以使用以下命令将 FetchContent 的源替换为从 gitee 镜像拉取再进行构建

```bash
source url_cn.bashrc
./build.sh $AIMRT_DOWNLOAD_FLAGS
```
