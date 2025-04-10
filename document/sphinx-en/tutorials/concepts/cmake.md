

# CMake in AimRT

AimRT uses native standard Modern CMake for building and manages dependencies through CMake FetchContent.

## Modern CMake
Modern CMake is the mainstream build method for contemporary C++ projects. It adopts an object-oriented approach by introducing concepts like Targets and properties, encapsulating all parameters including dependencies. This greatly simplifies dependency management and makes large system builds more organized and manageable. The AimRT framework uses Modern CMake for building, where each leaf directory represents a CMake Target. Cross-library references automatically handle all underlying dependencies.

For detailed Modern CMake usage, please refer to these resources:
- [CMake official website](https://cmake.org/cmake/help/latest/command/add_library.html)
- [More Modern CMake](https://hsf-training.github.io/hsf-training-cmake-webpage/aio/index.html)
- [Effective Modern CMake](https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1)
- [An Introduction to Modern CMake](https://cliutils.gitlab.io/modern-cmake/)

## Third-party Dependency Management in AimRT
When building AimRT from source, you should understand its dependency management strategy:
- AimRT uses CMake FetchContent to fetch dependencies. For details, refer to [CMake official documentation](https://cmake.org/cmake/help/latest/module/FetchContent.html).
- By default, dependencies are downloaded from their official sources. To use custom download URLs, modify `-DXXX_DOWNLOAD_URL` parameters during build (refer to `cmake/GetXXX.cmake`).
- For offline environments, set `-DXXX_LOCAL_SOURCE` parameters to specify local dependency paths (refer to `cmake/GetXXX.cmake`).
- Advanced users can modify `cmake/GetXXX.cmake` directly, as long as required CMake Targets are properly introduced.
- Note: AimRT only verifies default dependency versions. Compatibility and stability are your responsibility when modifying versions.

## Download Optimization
Most dependencies are downloaded from GitHub by default. For large dependencies like Boost that may encounter network issues, AimRT provides alternative Gitee mirrors.

On Linux platforms, use {{ '[url_cn.bashrc]({}/url_cn.bashrc)'.format(code_site_root_path_url) }} to set `AIMRT_DOWNLOAD_FLAGS` environment variable for alternative sources:

```bash
# Set AIMRT_DOWNLOAD_FLAGS to download from mirror site
source url_cn.bashrc
# Add AIMRT_DOWNLOAD_FLAGS to cmake generate command
cmake -Bbuild ... $AIMRT_DOWNLOAD_FLAGS
```
```bash
export AIMRT_DOWNLOAD_FLAGS="-DFMT_DOWNLOAD_URL=xxx ..."
cmake .. ${AIMRT_DOWNLOAD_FLAGS}
```

## CMake Options
AimRT consists of interface layer, runtime core, and various plugins/tools. Build configuration options:

| CMake Option Name                     | Type | Default | Description                                      |
| ------------------------------------- | ---- | ------- | ------------------------------------------------ |
| AIMRT_BUILD_TESTS                     | BOOL | OFF     | Whether to build tests                           |
| AIMRT_BUILD_PROTOCOLS                 | BOOL | ON      | Build AimRT protocols                            |
| AIMRT_BUILD_EXAMPLES                  | BOOL | OFF     | Build examples                                   |
| AIMRT_BUILD_DOCUMENT                  | BOOL | OFF     | Build documentation                              |
| AIMRT_BUILD_RUNTIME                   | BOOL | ON      | Build runtime core                               |
| AIMRT_BUILD_CLI_TOOLS                 | BOOL | OFF     | Build CLI tools                                  |
| AIMRT_BUILD_PYTHON_RUNTIME            | BOOL | OFF     | Build Python runtime                             |
| AIMRT_USE_FMT_LIB                     | BOOL | ON      | Use Fmt library (OFF uses std::format)           |
| AIMRT_BUILD_WITH_PROTOBUF             | BOOL | ON      | Use Protobuf library                             |
| AIMRT_USE_LOCAL_PROTOC_COMPILER       | BOOL | OFF     | Use local protoc compiler                        |
| AIMRT_USE_PROTOC_PYTHON_PLUGIN        | BOOL | OFF     | Use Python protoc plugin                         |
| AIMRT_BUILD_WITH_ROS2                 | BOOL | OFF     | Enable ROS2 Humble support                       |
| AIMRT_BUILD_NET_PLUGIN                | BOOL | OFF     | Build Net plugin                                 |
| AIMRT_BUILD_ROS2_PLUGIN               | BOOL | OFF     | Build ROS2 Humble plugin                         |
| AIMRT_BUILD_MQTT_PLUGIN               | BOOL | OFF     | Build MQTT plugin                                |
| AIMRT_BUILD_ZENOH_PLUGIN              | BOOL | OFF     | Build Zenoh plugin                               |
| AIMRT_BUILD_ICEORYX_PLUGIN            | BOOL | OFF     | Build Iceoryx plugin                             |
| AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN    | BOOL | OFF     | Build record/playback plugin                     |
| AIMRT_BUILD_TIME_MANIPULATOR_PLUGIN   | BOOL | OFF     | Build time manipulator plugin                    |
| AIMRT_BUILD_PARAMETER_PLUGIN          | BOOL | OFF     | Build parameter plugin                           |
| AIMRT_BUILD_LOG_CONTROL_PLUGIN        | BOOL | OFF     | Build log control plugin                         |
| AIMRT_BUILD_TOPIC_LOGGER_PLUGIN       | BOOL | OFF     | Build topic logger plugin                        |
| AIMRT_BUILD_OPENTELEMETRY_PLUGIN      | BOOL | OFF     | Build OpenTelemetry plugin                       |
| AIMRT_BUILD_GRPC_PLUGIN               | BOOL | OFF     | Build gRPC plugin                               |
| AIMRT_INSTALL                         | BOOL | ON      | Install AimRT                                    |
| AIMRT_BUILD_PYTHON_PACKAGE            | BOOL | OFF     | Build aimrt-py wheel package                     |

## CMake Targets
Available non-protocol CMake Targets in AimRT:

| CMake Target Name                                  | Description                          | Required Options                                                                 |
| -------------------------------------------------- | ------------------------------------ | -------------------------------------------------------------------------------- |
| aimrt::common::util                                | Basic utilities (string, log, etc.)  |                                                                                  |
| aimrt::common::net                                 | Network utilities (Boost Asio/Beast) | AIMRT_BUILD_RUNTIME, AIMRT_BUILD_NET_PLUGIN or AIMRT_BUILD_GRPC_PLUGIN           |
| aimrt::common::ros2_util                           | ROS2 utilities                       | AIMRT_BUILD_WITH_ROS2                                                            |
| aimrt::interface::aimrt_module_c_interface         | Module C interface                   |                                                                                  |
| aimrt::interface::aimrt_module_cpp_interface       | Module C++ interface                 |                                                                                  |
| aimrt::interface::aimrt_module_protobuf_interface  | Protobuf module interface            | AIMRT_BUILD_WITH_PROTOBUF                                                        |
| aimrt::interface::aimrt_module_ros2_interface      | ROS2 module interface                | AIMRT_BUILD_WITH_ROS2                                                            |
| aimrt::interface::aimrt_pkg_c_interface            | Package C interface                  |                                                                                  |
| aimrt::interface::aimrt_core_plugin_interface      | Plugin interface                     | AIMRT_BUILD_RUNTIME                                                              |
| aimrt::interface::aimrt_type_support_pkg_c_interface | Type support interface               |                                                                                  |
| aimrt::runtime::core                               | Runtime core library                 | AIMRT_BUILD_RUNTIME                                                              |