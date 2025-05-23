# Windows Source Code Build

Note that some plugins on the Windows platform have not been thoroughly tested. Their compilation options are disabled by default in build.bat, and enabling them may cause issues. If you encounter any problems, please provide feedback promptly.

## Required Dependencies

### MSVC Compilation Suite

The currently supported MSVC compilation suite version for AimRT is 1940. It is recommended to directly install [Visual Studio 2022](https://visualstudio.microsoft.com/zh-hans/downloads/) or later versions, and check the C++ desktop development module during installation.

Note that the MSVC toolset in Visual Studio 2022 was updated to version 19.40 in May 2024. Earlier versions with MSVC toolsets below 19.40 may encounter compilation issues.

### CMake

The minimum supported CMake version for AimRT is 3.24. You can download the appropriate version from the [CMake official website](https://cmake.org/download/):

## Minimal Build

After installing the above components, you can proceed with a minimal build without external dependencies. The build options are shown below:

```shell
cmake -B build ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DAIMRT_INSTALL=ON ^
    -DCMAKE_INSTALL_PREFIX="./build/install" ^
    -DAIMRT_BUILD_TESTS=OFF ^
    -DAIMRT_BUILD_EXAMPLES=ON ^
    -DAIMRT_BUILD_PROTOCOLS=ON ^
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

## Optional Dependencies

### Python and Related Packages

On the Windows platform, AimRT supports Python versions starting from 3.11. It is recommended to use [Python 3.11](https://www.python.org/downloads/release/python-31110/) or later versions.

After installation, you need to add Python to the system environment variables.

The airmt_cli tool depends on Python 3 and three libraries: pyinstaller, jinja2, and pyyaml. You can install them using the following command:

```shell
pip install pyinstaller jinja2 pyyaml --upgrade
```

The functionality to generate aimrt_py wheel packages depends on Python 3 and libraries such as build, setuptools, and wheel. You can install them using the following command:

```shell
pip install build setuptools wheel --upgrade
```

The above content corresponds to the following options:

```shell
-DAIMRT_BUILD_PYTHON_RUNTIME=ON
-DAIMRT_BUILD_CLI_TOOLS=ON
-DAIMRT_BUILD_PYTHON_PACKAGE=ON
```

## Full Build

This full build does not include plugins that have not been thoroughly tested. You can directly run build.bat to proceed.

```shell
./build.bat
```