# CMake in AimRT

AimRT uses native standard Modern CMake for building and relies on CMake FetchContent to manage dependencies.

## Modern CMake
Modern CMake is the mainstream build approach for current C++ projects. It adopts an object-oriented construction philosophy, introducing concepts like Targets and properties, which encapsulate all parameter information including dependencies. This greatly simplifies dependency management and makes building large systems more organized and effortless. The AimRT framework utilizes Modern CMake for construction, where each leaf folder represents a CMake Target. When libraries reference each other, all subordinate dependencies are automatically handled.

Detailed usage of Modern CMake won't be elaborated here. You may refer to other tutorials:
- [CMake official website](https://cmake.org/cmake/help/latest/command/add_library.html)
- [More Modern CMake](https://hsf-training.github.io/hsf-training-cmake-webpage/aio/index.html)
- [Effective Modern CMake](https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1)
- [An Introduction to Modern CMake](https://cliutils.gitlab.io/modern-cmake/)

## AimRT's Third-party Dependency Management Strategy

When you intend to reference/build AimRT from source, you need to understand its third-party dependency management strategy:
- AimRT uses CMake FetchContent to fetch dependencies. For detailed usage of CMake FetchContent, refer to the [CMake official documentation](https://cmake.org/cmake/help/latest/module/FetchContent.html).
- By default, AimRT downloads dependencies from their official sources. If you wish to use custom download URLs, examine the code in `cmake/GetXXX.cmake` and pass the `-DXXX_DOWNLOAD_URL` parameter during build to modify the download URL to your custom address.
- If your build environment lacks external network connectivity, you can download these dependencies offline. Then, reference the code in `cmake/GetXXX.cmake` and pass the `-DXXX_LOCAL_SOURCE` parameter during build to redirect the dependency search path to your specified local address.
- If the above methods still don't meet your customization needs for dependency management, you can directly modify the code in `cmake/GetXXX.cmake`, as long as it introduces the CMake Targets required for AimRT's build.
- Note: AimRT only validates the versions of third-party dependencies configured with default parameters. If you need to upgrade or downgrade these dependencies, ensure compatibility and stability yourself.

During the build process, most AimRT dependencies are downloaded from GitHub by default. Some dependencies like Boost are large in size. If download failures occur due to network issues, consider using the gitee mirror URLs provided by AimRT.

On Linux platforms, you can reference the {{ '[url_cn.bashrc]({}/url_cn.bashrc)'.format(code_site_root_path_url) }} file to set alternative download addresses as environment variables `AIMRT_DOWNLOAD_FLAGS`. Users can add this environment variable during CMake generation to download from mirror addresses.

To use mirror download addresses, first execute `source url_cn.bashrc`, then add the `AIMRT_DOWNLOAD_FLAGS` environment variable during CMake generation:
```bash
# Set AIMRT_DOWNLOAD_FLAGS to download from mirror site
source url_cn.bashrc
# Add AIMRT_DOWNLOAD_FLAGS to cmake generate command
cmake -Bbuild ... $AIMRT_DOWNLOAD_FLAGS
```

## AimRT's CMake Options
The AimRT framework consists of its interface layer, runtime core, plus multiple plugins and tools. During build, you can configure CMake options to select partial or complete components for compilation. The detailed CMake options are as follows:

| CMake Option Name                    | Type | Default | Description                                      |
| ----------------------------------- | ---- | ------ | ------------------------------------------------ |
| AIMRT_BUILD_TESTS                   | BOOL | OFF    | Whether to compile tests                         |
| AIMRT_BUILD_PROTOCOLS               | BOOL | ON     | Whether to compile protocols provided by AimRT   |
| AIMRT_BUILD_EXAMPLES                | BOOL | OFF    | Whether to compile examples                      |
| AIMRT_BUILD_DOCUMENT                | BOOL | OFF    | Whether to build documentation                   |
| AIMRT_BUILD_RUNTIME                 | BOOL | ON     | Whether to compile the runtime                   |
| AIMRT_BUILD_CLI_TOOLS               | BOOL | OFF    | Whether to compile CLI tools                     |
| AIMRT_BUILD_PYTHON_RUNTIME          | BOOL | OFF    | Whether to compile Python runtime                |
| AIMRT_USE_FMT_LIB                   | BOOL | ON     | Whether to use Fmt library (OFF uses std::format) |
| AIMRT_BUILD_WITH_PROTOBUF           | BOOL | ON     | Whether to use Protobuf library                  |
| AIMRT_USE_LOCAL_PROTOC_COMPILER     | BOOL | OFF    | Whether to use local protoc tool                 |
| AIMRT_USE_PROTOC_PYTHON_PLUGIN      | BOOL | OFF    | Whether to use Python version protoc plugin      |
| AIMRT_BUILD_WITH_ROS2               | BOOL | OFF    | Whether to use ROS2 Humble                       |
| AIMRT_BUILD_NET_PLUGIN              | BOOL | OFF    | Whether to compile Net plugin                    |
| AIMRT_BUILD_ROS2_PLUGIN             | BOOL | OFF    | Whether to compile ROS2 Humble plugin            |
| AIMRT_BUILD_MQTT_PLUGIN             | BOOL | OFF    | Whether to compile MQTT plugin                   |
| AIMRT_BUILD_ZENOH_PLUGIN            | BOOL | OFF    | Whether to compile Zenoh plugin                  |
| AIMRT_BUILD_ICEORYX_PLUGIN          | BOOL | OFF    | Whether to compile Iceoryx plugin                |
| AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN  | BOOL | OFF    | Whether to compile record/playback plugin        |
| AIMRT_BUILD_TIME_MANIPULATOR_PLUGIN | BOOL | OFF    | Whether to compile time manipulator plugin       |
| AIMRT_BUILD_PARAMETER_PLUGIN        | BOOL | OFF    | Whether to compile parameter plugin              |
| AIMRT_BUILD_LOG_CONTROL_PLUGIN      | BOOL | OFF    | Whether to compile log control plugin            |
| AIMRT_BUILD_TOPIC_LOGGER_PLUGIN     | BOOL | OFF    | Whether to compile topic logger plugin           |
| AIMRT_BUILD_OPENTELEMETRY_PLUGIN    | BOOL | OFF    | Whether to compile OpenTelemetry plugin          |
| AIMRT_BUILD_GRPC_PLUGIN             | BOOL | OFF    | Whether to compile gRPC plugin                   |
| AIMRT_INSTALL                       | BOOL | ON     | Whether to install AimRT                         |
| AIMRT_BUILD_PYTHON_PACKAGE          | BOOL | OFF    | Whether to compile aimrt-py wheel package        |

## CMake Targets in AimRT
All referenceable non-protocol CMake Targets in AimRT are as follows:

| CMake Target Name                                     | Description                                  | Required Macros                                                                 |
| ---------------------------------------------------- | ----------------------------------------- | ---------------------------------------------------------------------- |
| aimrt::common::util                                  | Independent basic utilities (string, log, etc.) |                                                                        |
| aimrt::common::net                                   | Independent network utilities (based on Boost ASIO/Beast) | AIMRT_BUILD_RUNTIME, AIMRT_BUILD_NET_PLUGIN or AIMRT_BUILD_GRPC_PLUGIN |
| aimrt::common::ros2_util                             | Independent ROS2-related basic utilities   | AIMRT_BUILD_WITH_ROS2                                                  |
| aimrt::interface::aimrt_module_c_interface           | Module development interface (C version)   |                                                                        |
| aimrt::interface::aimrt_module_cpp_interface         | Module development interface (C++ version) |                                                                        |
| aimrt::interface::aimrt_module_protobuf_interface    | Protobuf-related module interface (based on C++ interface) | AIMRT_BUILD_WITH_PROTOBUF                                              |
| aimrt::interface::aimrt_module_ros2_interface        | ROS2-related module interface (based on C++ interface) | AIMRT_BUILD_WITH_ROS2                                                  |
| aimrt::interface::aimrt_pkg_c_interface              | Package development interface              |                                                                        |
| aimrt::interface::aimrt_core_plugin_interface        | Plugin development interface               | AIMRT_BUILD_RUNTIME                                                    |
| aimrt::interface::aimrt_type_support_pkg_c_interface | Type support interface                     |                                                                        |
| aimrt::runtime::core                                 | Runtime core library                       | AIMRT_BUILD_RUNTIME                                                    |