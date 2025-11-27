
# AimRT 中的 CMake


AimRT 使用原生标准的 Modern CMake 进行构建，并基于 CMake Fetchcontent 获取依赖。

## Modern CMake
Modern CMake 是目前主流 C++ 项目的常用构建方式，使用面向对象的构建思路，引入了 Target、属性等概念，可以将包括依赖在内的各个参数信息全部封装起来，大大简化了依赖管理，使构建大型系统更有条理、更加轻松。AimRT 框架使用 Modern CMake 进行构建，每个叶子文件夹是一个 CMake Target。库之间相互引用时能自动处理下层级的所有依赖。

关于 Modern CMake 详细使用方式此处不做赘述，您可以参考一些其他教程：
- [CMake official website](https://cmake.org/cmake/help/latest/command/add_library.html)
- [More Modern CMake](https://hsf-training.github.io/hsf-training-cmake-webpage/aio/index.html)
- [Effective Modern CMake](https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1)
- [An Introduction to Modern CMake](https://cliutils.gitlab.io/modern-cmake/)

## AimRT 的第三方依赖管理策略

当您准备从源码引用/构建 AimRT 时，您需要了解 AimRT 第三方依赖管理策略：
- AimRT 使用 CMake FetchContent 拉取依赖，关于 CMake FetchContent 的详细使用方式，请参考[CMake官方文档](https://cmake.org/cmake/help/latest/module/FetchContent.html)。
- AimRT 默认从各个第三方依赖的官方下载地址进行下载，如果你想通过自定义的下载地址下载这些依赖，可以参考`cmake/GetXXX.cmake`中的代码，在构建时传入`-DXXX_DOWNLOAD_URL`参数，将下载url修改为您自定义的地址。
- 如果您的构建环境无法连接外部网络，你也可以离线的下载这些依赖，然后参考`cmake/GetXXX.cmake`中的代码，在构建时传入`-DXXX_LOCAL_SOURCE`参数，将依赖寻找地址转为您指定的本地地址。
- 如果以上方式还不满足您对依赖管理的自定义需求，您也可以直接自定义`cmake/GetXXX.cmake`中的代码，只要引入满足 AimRT 构建所需的 CMake Target 即可。
- 请注意，AimRT 仅验证了默认参数中配置的各个第三方依赖的版本，如果您需要升级或降级这些第三方依赖的版本，请自行保证兼容性和稳定性。


AimRT 在构建过程中，大部分依赖都默认从 github 下载，其中部分依赖如 Boost 体积较大，如果由于网络问题下载失败，可以考虑使用 AimRT 提供的 gitee 备选下载地址。

在 linux 平台上，可以参考 {{ '[url_cn.bashrc]({}/url_cn.bashrc)'.format(code_site_root_path_url) }} 文件，将备选下载地址设置为环境变量 `AIMRT_DOWNLOAD_FLAGS`，用户可以在 CMake 生成命令中添加该环境变量以从备选地址下载。

要使用备用下载地址，需要首先 `source url_cn.bashrc`，然后 CMake 生成时添加 `AIMRT_DOWNLOAD_FLAGS` 环境变量：
```bash
# Set AIMRT_DOWNLOAD_FLAGS to download from mirror site
source url_cn.bashrc
# Add AIMRT_DOWNLOAD_FLAGS to cmake generate command
cmake -Bbuild ... $AIMRT_DOWNLOAD_FLAGS
```


## AimRT 的 CMake 选项
AimRT 框架由其 interface 层、runtime 主体，加上多个插件、工具共同组成，在构建时可以通过配置 CMake 选项，选择其中一部分或全部进行构建。详细的 CMake 选项列表如下：

| CMake Option名称                    | 类型 | 默认值 | 作用                                             |
| ----------------------------------- | ---- | ------ | ------------------------------------------------ |
| AIMRT_BUILD_TESTS                   | BOOL | OFF    | 是否编译测试                                     |
| AIMRT_BUILD_PROTOCOLS               | BOOL | ON     | 是否编译 AimRT 提供的协议                        |
| AIMRT_BUILD_EXAMPLES                | BOOL | OFF    | 是否编译示例                                     |
| AIMRT_BUILD_DOCUMENT                | BOOL | OFF    | 是否构建文档                                     |
| AIMRT_BUILD_RUNTIME                 | BOOL | ON     | 是否编译运行时                                   |
| AIMRT_BUILD_CLI_TOOLS               | BOOL | OFF    | 是否编译 cli 工具                                |
| AIMRT_BUILD_PYTHON_RUNTIME          | BOOL | OFF    | 是否编译 Python 运行时                           |
| AIMRT_USE_FMT_LIB                   | BOOL | ON     | 是否使用 Fmt 库，如果设为 OFF 将使用 std::format |
| AIMRT_BUILD_WITH_PROTOBUF           | BOOL | ON     | 是否使用 Protobuf 库                             |
| AIMRT_USE_LOCAL_PROTOC_COMPILER     | BOOL | OFF    | 是否使用本地的 protoc 工具                       |
| AIMRT_USE_PROTOC_PYTHON_PLUGIN      | BOOL | OFF    | 是否使用 Python 版本 protoc 插件                 |
| AIMRT_BUILD_WITH_ROS2               | BOOL | OFF    | 是否使用 ROS2 Humble                             |
| AIMRT_BUILD_NET_PLUGIN              | BOOL | OFF    | 是否编译 Net 插件                                |
| AIMRT_BUILD_ROS2_PLUGIN             | BOOL | OFF    | 是否编译 ROS2 Humble 插件                        |
| AIMRT_BUILD_MQTT_PLUGIN             | BOOL | OFF    | 是否编译 Mqtt 插件                               |
| AIMRT_BUILD_ZENOH_PLUGIN            | BOOL | OFF    | 是否编译 Zenoh 插件                              |
| AIMRT_BUILD_ICEORYX_PLUGIN          | BOOL | OFF    | 是否编译 Iceoryx 插件                            |
| AIMRT_BUILD_RECORD_PLAYBACK_PLUGIN  | BOOL | OFF    | 是否编译录播插件                                 |
| AIMRT_BUILD_TIME_MANIPULATOR_PLUGIN | BOOL | OFF    | 是否编译 time manipulator 插件                   |
| AIMRT_BUILD_PARAMETER_PLUGIN        | BOOL | OFF    | 是否编译 parameter 插件                          |
| AIMRT_BUILD_LOG_CONTROL_PLUGIN      | BOOL | OFF    | 是否编译日志控制插件                             |
| AIMRT_BUILD_TOPIC_LOGGER_PLUGIN     | BOOL | OFF    | 是否编译话题日志插件                             |
| AIMRT_BUILD_OPENTELEMETRY_PLUGIN    | BOOL | OFF    | 是否编译 opentelemetry 插件                      |
| AIMRT_BUILD_GRPC_PLUGIN             | BOOL | OFF    | 是否编译 grpc 插件                               |
| AIMRT_BUILD_ECHO_PLUGIN             | BOOL | OFF    | 是否编译 echo 插件                               |
| AIMRT_INSTALL                       | BOOL | ON     | 是否需要 install aimrt                           |
| AIMRT_BUILD_PYTHON_PACKAGE          | BOOL | OFF    | 是否编译 aimrt-py whl包                          |
| AIMRT_EXECUTOR_USE_STDEXEC          | BOOL | OFF    | 是否使用 stdexec 为协程执行器实现，设为 OFF 将使用 libunifex |
| AIMRT_ENABLE_DLOPEN_DEEPBIND        | BOOL | ON     | 加载动态库时是否附带 `RTLD_DEEPBIND`              |


## AimRT 中的 CMake Target
AimRT 中所有的可引用的非协议类型 CMake Target 如下：

| CMake Target名称                                     | 作用                                      | 需要开启的宏                                                           |
| ---------------------------------------------------- | ----------------------------------------- | ---------------------------------------------------------------------- |
| aimrt::common::util                                  | 一些独立基础工具，如 string、log 等       |                                                                        |
| aimrt::common::net                                   | 一些独立的网络工具，基于 boost asio/beast | AIMRT_BUILD_RUNTIME、AIMRT_BUILD_NET_PLUGIN 或 AIMRT_BUILD_GRPC_PLUGIN |
| aimrt::common::ros2_util                             | 独立的 ros2 相关的基础工具                | AIMRT_BUILD_WITH_ROS2                                                  |
| aimrt::interface::aimrt_module_c_interface           | 模块开发接口-C 版本                       |                                                                        |
| aimrt::interface::aimrt_module_cpp_interface         | 模块开发接口-CPP 版本                     |                                                                        |
| aimrt::interface::aimrt_module_protobuf_interface    | 模块开发 protobuf 相关接口，基于 CPP 接口 | AIMRT_BUILD_WITH_PROTOBUF                                              |
| aimrt::interface::aimrt_module_ros2_interface        | 模块开发 ros2 相关接口，基于 CPP 接口     | AIMRT_BUILD_WITH_ROS2                                                  |
| aimrt::interface::aimrt_pkg_c_interface              | Pkg 开发接口                              |                                                                        |
| aimrt::interface::aimrt_core_plugin_interface        | 插件开发接口                              | AIMRT_BUILD_RUNTIME                                                    |
| aimrt::interface::aimrt_type_support_pkg_c_interface | type support 接口                         |                                                                        |
| aimrt::runtime::core                                 | 运行时核心库                              | AIMRT_BUILD_RUNTIME                                                    |

