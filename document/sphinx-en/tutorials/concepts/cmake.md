# CMake in AimRT

AimRT uses native standard Modern CMake for building and relies on CMake FetchContent to obtain dependencies.

## Modern CMake

Modern CMake is the mainstream build method for current C++ projects. It adopts an object-oriented build approach, introducing concepts such as Target and properties, which can encapsulate all parameter information including dependencies. This greatly simplifies dependency management, making the construction of large systems more organized and easier. The AimRT framework is built using Modern CMake, where each leaf folder is a CMake Target. When libraries reference each other, all lower-level dependencies are automatically handled.

For detailed usage of Modern CMake, we will not elaborate here. You can refer to some other tutorials:

- [CMake official website](https://cmake.org/cmake/help/latest/command/add_library.html)
- [More Modern CMake](https://hsf-training.github.io/hsf-training-cmake-webpage/aio/index.html)
- [Effective Modern CMake](https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1)
- [An Introduction to Modern CMake](https://cliutils.gitlab.io/modern-cmake/)

## AimRT's Third-Party Dependency Management Strategy

When you plan to reference/build AimRT from source, you need to understand AimRT's third-party dependency management strategy:

- AimRT uses CMake FetchContent to pull dependencies. For detailed usage of CMake FetchContent, please refer to [CMake official documentation](https://cmake.org/cmake/help/latest/module/FetchContent.html).
- AimRT defaults to downloading from the official download addresses of each third-party dependency. If you want to download these dependencies through a custom download address, you can refer to the code in `cmake/GetXXX.cmake`, and pass the `-DXXX_DOWNLOAD_URL` parameter during the build to change the download URL to your custom address.
- If your build environment cannot connect to the external network, you can also download these dependencies offline, then refer to the code in `cmake/GetXXX.cmake`, and pass the `-DXXX_LOCAL_SOURCE` parameter during the build to change the dependency search path to your specified local address.
- If the above methods still do not meet your custom dependency management needs, you can also directly customize the code in `cmake/GetXXX.cmake`, as long as the CMake Target required for AimRT's build is introduced.
- Please note that AimRT only verifies the versions of each third-party dependency configured in the default parameters. If you need to upgrade or downgrade these third-party dependency versions, please ensure compatibility and stability on your own.

During the build process, most dependencies in AimRT are downloaded from github by default. Some dependencies, such as Boost, are large in size. If the download fails due to network issues, you can consider using the gitee alternative download address provided by AimRT.

On the linux platform, you can refer to the {{ '[url_cn.bashrc]({}/url_cn.bashrc)'.format(code_site_root_path_url) }} file to set the alternative download address as the environment variable `AIMRT_DOWNLOAD_FLAGS`. Users can add this environment variable to the CMake generation command to download from the alternative address.

To use the alternative download address, you need to first `source url_cn.bashrc`, then add the `AIMRT_DOWNLOAD_FLAGS` environment variable when generating with CMake:

```bash
# Set AIMRT_DOWNLOAD_FLAGS to download from mirror site
source url_cn.bashrc
# Add AIMRT_DOWNLOAD_FLAGS to cmake generate command
cmake -Bbuild ... $AIMRT_DOWNLOAD_FLAGS
```

## AimRT's CMake Options

The AimRT framework consists of its interface layer, runtime core, plus multiple plugins and tools. During the build, you can choose to build some or all of them by configuring CMake options. The detailed list of CMake options is as follows:

| CMake Option Name                   | Type | Default | Function                                                                                               |
| ----------------------------------- | ---- | ------- | ------------------------------------------------------------------------------------------------------ |
| AIMRT_BUILD_TESTS                   | BOOL | OFF     | Whether to compile tests                                                                               |
| AIMRT_BUILD_PROTOCOLS               | BOOL | ON      | Whether to compile protocols provided by AimRT                                                         |
| AIMRT_BUILD_EXAMPLES                | BOOL | OFF     | Whether to compile examples                                                                            |
| AIMRT_BUILD_DOCUMENT                | BOOL | OFF     | Whether to build documentation                                                                         |
| AIMRT_BUILD_RUNTIME                 | BOOL | ON      | Whether to compile the runtime                                                                         |
| AIMRT_BUILD_CLI_TOOLS               | BOOL | OFF     | Whether to compile cli tools                                                                           |
| AIMRT_BUILD_PYTHON_RUNTIME          | BOOL | OFF     | Whether to compile the Python runtime                                                                  |
| AIMRT_USE_FMT_LIB                   | BOOL | ON      | Whether to use the Fmt library, if set to OFF, std::format will be used                                |
| AIMRT_BUILD_WITH_PROTOBUF           | BOOL | ON      | Whether to use the Protobuf library                                                                    |
| AIMRT_USE_LOCAL_PROTOC_COMPILER     | BOOL | OFF     | Whether to use the local protoc tool                                                                   |
| AIMRT_USE_PROTOC_PYTHON_PLUGIN      | BOOL | OFF     | Whether to use the Python version protoc plugin                                                        |
| AIMRT_BUILD_WITH_ROS2               | BOOL | OFF     | Whether to use ROS2 Humble                                                                             |
| AIMRT_BUILD_NET_PLUGIN              | BOOL | OFF     | Whether to compile the Net plugin                                                                      |
| AIMRT_BUILD_ROS2_PLUGIN             | BOOL | OFF     | Whether to compile the ROS2 Humble plugin                                                              |
| AIMRT_BUILD_MQTT_PLUGIN             | BOOL | OFF     | Whether to compile the Mqtt plugin                                                                     |
| AIMRT_BUILD_ZENOH_PLUGIN            | BOOL | OFF     | Whether to compile the Zenoh plugin                                                                    |
| AIMRT_BUILD_ICEORYX_PLUGIN          | BOOL | OFF     | Whether to compile the Iceoryx plugin                                                                  |
| AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN  | BOOL | OFF     | Whether to compile the record and playback plugin                                                      |
| AIMRT_BUILD_TIME_MANIPULATOR_PLUGIN | BOOL | OFF     | Whether to compile the time manipulator plugin                                                         |
| AIMRT_BUILD_PARAMETER_PLUGIN        | BOOL | OFF     | Whether to compile the parameter plugin                                                                |
| AIMRT_BUILD_LOG_CONTROL_PLUGIN      | BOOL | OFF     | Whether to compile the log control plugin                                                              |
| AIMRT_BUILD_TOPIC_LOGGER_PLUGIN     | BOOL | OFF     | Whether to compile the topic logger plugin                                                             |
| AIMRT_BUILD_OPENTELEMETRY_PLUGIN    | BOOL | OFF     | Whether to compile the opentelemetry plugin                                                            |
| AIMRT_BUILD_GRPC_PLUGIN             | BOOL | OFF     | Whether to compile the grpc plugin                                                                     |
| AIMRT_BUILD_ECHO_PLUGIN             | BOOL | OFF     | Whether to compile the echo plugin                                                                     |
| AIMRT_INSTALL                       | BOOL | ON      | Whether to install aimrt                                                                               |
| AIMRT_BUILD_PYTHON_PACKAGE          | BOOL | OFF     | Whether to compile the aimrt-py whl package                                                            |
| AIMRT_EXECUTOR_USE_STDEXEC          | BOOL | OFF     | Whether to use stdexec as the coroutine executor implementation; if set to OFF, libunifex will be used |
| AIMRT_ENABLE_DLOPEN_DEEPBIND        | BOOL | ON      | Whether to use `RTLD_DEEPBIND` when loading dynamic libraries                                          |

## CMake Targets in AimRT

All referencable non-protocol type CMake Targets in AimRT are as follows:

| CMake Target Name                                    | Function                                                              | Macros to Enable                                                        |
| ---------------------------------------------------- | --------------------------------------------------------------------- | ----------------------------------------------------------------------- |
| aimrt::common::util                                  | Some independent basic tools, such as string, log, etc.               |                                                                         |
| aimrt::common::net                                   | Some independent network tools, based on boost asio/beast             | AIMRT_BUILD_RUNTIME, AIMRT_BUILD_NET_PLUGIN, or AIMRT_BUILD_GRPC_PLUGIN |
| aimrt::common::ros2_util                             | Independent ros2 related basic tools                                  | AIMRT_BUILD_WITH_ROS2                                                   |
| aimrt::interface::aimrt_module_c_interface           | Module development interface - C version                              |                                                                         |
| aimrt::interface::aimrt_module_cpp_interface         | Module development interface - CPP version                            |                                                                         |
| aimrt::interface::aimrt_module_protobuf_interface    | Module development protobuf related interface, based on CPP interface | AIMRT_BUILD_WITH_PROTOBUF                                               |
| aimrt::interface::aimrt_module_ros2_interface        | Module development ros2 related interface, based on CPP interface     | AIMRT_BUILD_WITH_ROS2                                                   |
| aimrt::interface::aimrt_pkg_c_interface              | Pkg development interface                                             |                                                                         |
| aimrt::interface::aimrt_core_plugin_interface        | Plugin development interface                                          | AIMRT_BUILD_RUNTIME                                                     |
| aimrt::interface::aimrt_type_support_pkg_c_interface | type support interface                                                |                                                                         |
| aimrt::runtime::core                                 | Runtime core library                                                  | AIMRT_BUILD_RUNTIME                                                     |
