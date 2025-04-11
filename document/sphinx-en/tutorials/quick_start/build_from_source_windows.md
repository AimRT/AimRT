

# Windows Source Build

Note that some plugins on the Windows platform have not been thoroughly tested. Their compilation options are disabled by default in build.bat, and enabling them may cause issues. Please report any problems encountered.

## Required Dependencies

### MSVC Toolchain

AimRT currently supports MSVC toolchain version 1940. We recommend installing [Visual Studio 2022](https://visualstudio.microsoft.com/zh-hans/downloads/) or later, selecting the C++ desktop development workload during installation.


### CMake

The minimum supported CMake version for AimRT is 3.24. Download and install the appropriate version from the [CMake official website](https://cmake.org/download/):

## Minimal Build

After installing the above components, you can perform a minimal build without external dependencies using the following options:

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

AimRT on Windows supports Python version 3.11 or higher. We recommend using [Python 3.11](https://www.python.org/downloads/release/python-31110/) or newer.

After installation, add Python to the system environment variables.

The airmt_cli tool requires Python 3 and three libraries: pyinstaller, jinja2, and pyyaml. Install them using:

```shell
pip install pyinstaller jinja2 pyyaml --upgrade
```

The whl package generation functionality for aimrt_py requires Python 3 and libraries including build, setuptools, and wheel. Install them using:

```shell
pip install build setuptools wheel --upgrade
```

These requirements correspond to the following build options respectively:

```shell
-DAIMRT_BUILD_PYTHON_RUNTIME=ON
-DAIMRT_BUILD_CLI_TOOLS=ON
-DAIMRT_BUILD_PYTHON_PACKAGE=ON
```

## Full Build

This complete build excludes insufficiently tested plugins. Simply execute build.bat directly:

```shell
./build.bat
```