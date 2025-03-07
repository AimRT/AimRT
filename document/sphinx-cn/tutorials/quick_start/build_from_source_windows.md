
# Windows 源码构建

注意 Windows 平台下部分插件未经过实际测试，其编译选项在 build.bat 中默认关闭，启用后可能存在问题，如果遇到问题请及时反馈。

## 必选依赖

### MSVC 编译套件

当前 AimRT 支持的 MSVC 编译套件版本为 1940，推荐直接安装 [Visual Studio 2022](https://visualstudio.microsoft.com/zh-hans/downloads/) 及以上版本，安装时勾选 C++ 桌面开发模块。

注意 Visual Studio 2022 的 MSVC 工具集在 2024 年 5 月更新到了 19.40 版本， 早期版本低于 19.40 的 MSVC 工具集可能存在编译问题。

### CMake

当前 AimRT 支持的最低 CMake 版本为 3.24，可以通过 [CMake 官方网站](https://cmake.org/download/)下载合适的版本进行安装：


## 最小化构建

将以上内容进行安装后即可进行无外部依赖的最小化构建，构建选项如下所示

```shell
cmake -B build ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DAIMRT_INSTALL=ON ^
    -DCMAKE_INSTALL_PREFIX="./build/install" ^
    -DAIMRT_BUILD_TESTS=OFF ^
    -DAIMRT_BUILD_EXAMPLES=ON ^
    -DAIMRT_BUILD_DOCUMENT=ON ^
    -DAIMRT_BUILD_RUNTIME=ON ^
    -DAIMRT_BUILD_CLI_TOOLS=OFF ^
    -DAIMRT_BUILD_PYTHON_RUNTIME=OFF ^
    -DAIMRT_USE_FMT_LIB=ON ^
    -DAIMRT_BUILD_WITH_PROTOBUF=ON ^
    -DAIMRT_USE_LOCAL_PROTOC_COMPILER=OFF ^
    -DAIMRT_USE_PROTOC_PYTHON_PLUGIN=OFF ^
    -DAIMRT_BUILD_WITH_ROS2=OFF ^
    -DAIMRT_BUILD_NET_PLUGIN=ON ^
    -DAIMRT_BUILD_MQTT_PLUGIN=OFF ^
    -DAIMRT_BUILD_ZENOH_PLUGIN=OFF ^
    -DAIMRT_BUILD_ICEORYX_PLUGIN=OFF ^
    -DAIMRT_BUILD_ROS2_PLUGIN=OFF ^
    -DAIMRT_BUILD_RECORD_PLAYBACK_PLUGIN=ON ^
    -DAIMRT_BUILD_TIME_MANIPULATOR_PLUGIN=ON ^
    -DAIMRT_BUILD_PARAMETER_PLUGIN=ON ^
    -DAIMRT_BUILD_LOG_CONTROL_PLUGIN=ON ^
    -DAIMRT_BUILD_TOPIC_LOGGER_PLUGIN=ON ^
    -DAIMRT_BUILD_OPENTELEMETRY_PLUGIN=OFF ^
    -DAIMRT_BUILD_GRPC_PLUGIN=OFF ^
    -DAIMRT_BUILD_PYTHON_PACKAGE=OFF

cmake --build build --config Release --target all
```

## 可选依赖

### Python 及其相关包

AimRT 在 Windows 平台下支持的 Python 版本最低为 3.11，推荐使用 [Python 3.11](https://www.python.org/downloads/release/python-31110/) 及以上版本。

安装完成后需要将 Python 添加到系统环境变量中。

airmt_cli 工具依赖于 Python 3 以及 pyinstaller jinja2 pyyaml 三个库，可以通过以下命令进行安装

```shell
pip install pyinstaller jinja2 pyyaml --upgrade
```

打包生成 aimrt_py 的 whl 包功能依赖 Python 3 以及 build setuptools wheel 等库，可以通过以下命令进行安装

```shell
pip install build setuptools wheel --upgrade
```

以上内容分别对应以下选项

```shell
-DAIMRT_BUILD_PYTHON_RUNTIME=ON
-DAIMRT_BUILD_CLI_TOOLS=ON
-DAIMRT_BUILD_PYTHON_PACKAGE=ON
```

## 完整构建

此处完整构建不包含未经充分测试的插件，直接运行 build.bat 即可。

```shell
./build.bat
```
