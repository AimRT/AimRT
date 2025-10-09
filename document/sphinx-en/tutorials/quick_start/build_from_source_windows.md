# Windows Source Build

Note that some plugins on the Windows platform have not been thoroughly tested; their compilation options are disabled by default in build.bat. Enabling them may cause issues—please report any problems promptly.

## Required Dependencies

### MSVC Toolchain

AimRT currently supports MSVC toolchain version 1940. It is recommended to install [Visual Studio 2022](https://visualstudio.microsoft.com/downloads/) or later, selecting the **Desktop development with C++** workload during installation.

Note that the MSVC toolset in Visual Studio 2022 was updated to version 19.40 in May 2024. Earlier toolsets below 19.40 may encounter compilation issues.

### CMake

The minimum supported CMake version for AimRT is 3.24. Download an appropriate release from the [CMake official site](https://cmake.org/download/) and install it.

## Minimal Build

After installing the above, you can perform a minimal build without external dependencies. The build options are shown below:


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

On Windows, AimRT supports Python 3.11 or later. It is recommended to use [Python 3.11](https://www.python.org/downloads/release/python-31110/) or newer.

After installation, add Python to the system environment variables.

The `airmt_cli` tool depends on Python 3 and the packages `pyinstaller`, `jinja2`, and `pyyaml`. Install them with:


```shell
pip install pyinstaller jinja2 pyyaml --upgrade
```


Building the `aimrt_py` wheel package requires Python 3 along with `build`, `setuptools`, and `wheel`. Install them with:


```shell
pip install build setuptools wheel --upgrade
```


The above correspond to the following options:


```shell
-DAIMRT_BUILD_PYTHON_RUNTIME=ON
-DAIMRT_BUILD_CLI_TOOLS=ON
-DAIMRT_BUILD_PYTHON_PACKAGE=ON
```


## Full Build

The full build here excludes plugins that have not been sufficiently tested. Simply run build.bat.