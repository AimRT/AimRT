@echo off
setlocal

cmake -B build ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DAIMRT_INSTALL=ON ^
    -DCMAKE_INSTALL_PREFIX="./build/install" ^
    -DAIMRT_BUILD_TESTS=OFF ^
    -DAIMRT_BUILD_EXAMPLES=ON ^
    -DAIMRT_BUILD_DOCUMENT=ON ^
    -DAIMRT_BUILD_RUNTIME=ON ^
    -DAIMRT_BUILD_CLI_TOOLS=ON ^
    -DAIMRT_BUILD_BAGTRANS=ON ^
    -DAIMRT_BUILD_PYTHON_RUNTIME=ON ^
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
    -DAIMRT_BUILD_OPENTELEMETRY_PLUGIN=OFF ^
    -DAIMRT_BUILD_GRPC_PLUGIN=OFF ^
    -DAIMRT_BUILD_PYTHON_PACKAGE=ON ^
    %*

if %errorlevel% neq 0 (
    echo cmake failed
    exit /b 1
)

if exist .\build\install (
    rmdir /s /q .\build\install
)

cmake --build build --config Release --target install --parallel %NUMBER_OF_PROCESSORS%

if %errorlevel% neq 0 (
    echo build failed
    exit /b 1
)

endlocal
