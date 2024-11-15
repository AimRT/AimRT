# opentelemetry plugin examples



## rpc trace

一个基于 **opentelemetry_plugin** 的 rpc trace 示例，演示内容包括：
- 如何在启动时加载 **opentelemetry_plugin**；
- 如何为 rpc 配置 trace 功能；


核心代码：
- [rpc.proto](../../../protocols/example/rpc.proto)
- [normal_rpc_co_client_module.cc](../../cpp/pb_rpc/module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)
- [normal_rpc_co_server_module.cc](../../cpp/pb_rpc/module/normal_rpc_co_server_module/normal_rpc_co_server_module.cc)
- [service.cc](../../cpp/pb_rpc/module/normal_rpc_co_server_module/service.cc)


配置文件：
- [examples_plugins_opentelemetry_plugin_pb_rpc_trace_cfg.yaml](./install/linux/bin/cfg/examples_plugins_opentelemetry_plugin_pb_rpc_trace_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_OPENTELEMETRY_PLUGIN` 选项编译 AimRT；
- 将启动配置中的 `trace_otlp_http_exporter_url` 配置为 collector 或 jaejer 的上报地址，详情请参考插件文档；
- 直接运行 build 目录下`start_examples_plugins_opentelemetry_plugin_pb_rpc_trace.sh`脚本启动进程；
- 在 jaejer 平台上观察 rpc trace 数据；
- 键入`ctrl-c`停止进程；


说明：
- 此示例基于 protobuf rpc local 后端示例，通过 **opentelemetry_plugin** 中的 `otp_trace` 类型 filter 上报 rpc trace 数据；



## channel trace



一个基于 **opentelemetry_plugin** 的 channel trace 示例，演示内容包括：
- 如何在启动时加载 **opentelemetry_plugin**；
- 如何为 channel 配置 trace 功能；



核心代码：
- [event.proto](../../../protocols/example/event.proto)
- [normal_publisher_module.cc](../../cpp/pb_chn/module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](../../cpp/pb_chn/module/normal_subscriber_module/normal_subscriber_module.cc)



配置文件：
- [examples_plugins_opentelemetry_plugin_pb_chn_trace_cfg.yaml](./install/linux/bin/cfg/examples_plugins_opentelemetry_plugin_pb_chn_trace_cfg.yaml)





运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_OPENTELEMETRY_PLUGIN` 选项编译 AimRT；
- 将启动配置中的 `trace_otlp_http_exporter_url` 配置为 collector 或 jaejer 的上报地址，详情请参考插件文档；
- 直接运行 build 目录下`start_examples_plugins_opentelemetry_plugin_pb_chn_trace.sh`脚本启动进程；
- 在 jaejer 平台上观察 channel trace 数据；
- 键入`ctrl-c`停止进程；


说明：
- 此示例基于 protobuf channel local 后端示例，通过 **opentelemetry_plugin** 中的 `otp_trace` 类型 filter 上报 channel trace 数据；


## rpc metrics

一个基于 **opentelemetry_plugin** 的 rpc metrics 示例，演示内容包括：
- 如何在启动时加载 **opentelemetry_plugin**；
- 如何为 rpc 配置 metrics 功能；
- 如何设置 rpc 调用时间的直方图的边界；


核心代码：
- [rpc.proto](../../../protocols/example/rpc.proto)
- [normal_rpc_co_client_module.cc](../../cpp/pb_rpc/module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)
- [normal_rpc_co_server_module.cc](../../cpp/pb_rpc/module/normal_rpc_co_server_module/normal_rpc_co_server_module.cc)
- [service.cc](../../cpp/pb_rpc/module/normal_rpc_co_server_module/service.cc)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_OPENTELEMETRY_PLUGIN` 选项编译 AimRT；
- 将启动配置中的 `metrics_otlp_http_exporter_url` 配置为 collector 或 jaejer 的上报地址，详情请参考插件文档；
- 直接运行 build 目录下`start_examples_plugins_opentelemetry_plugin_pb_rpc_metrics.sh`脚本启动进程；
- 在 jaejer 平台上观察 rpc metrics 数据；
- 键入`ctrl-c`停止进程；

说明：
- 此示例基于 protobuf rpc local 后端示例，通过 **opentelemetry_plugin** 中的 `otp_metrics` 类型 filter 上报 rpc metrics 数据；



## channel metrics

一个基于 **opentelemetry_plugin** 的 channel metrics 示例，演示内容包括：
- 如何在启动时加载 **opentelemetry_plugin**；
- 如何为 channel 配置 metrics 功能；

核心代码：
- [event.proto](../../../protocols/example/event.proto)
- [normal_publisher_module.cc](../../cpp/pb_chn/module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](../../cpp/pb_chn/module/normal_subscriber_module/normal_subscriber_module.cc)


配置文件：
- [examples_plugins_opentelemetry_plugin_pb_chn_metrics_cfg.yaml](./install/linux/bin/cfg/examples_plugins_opentelemetry_plugin_pb_chn_metrics_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_OPENTELEMETRY_PLUGIN` 选项编译 AimRT；
- 将启动配置中的 `metrics_otlp_http_exporter_url` 配置为 collector 或 jaejer 的上报地址，详情请参考插件文档；
- 直接运行 build 目录下`start_examples_plugins_opentelemetry_plugin_pb_chn_metrics.sh`脚本启动进程；
- 在 jaejer 平台上观察 channel metrics 数据；
- 键入`ctrl-c`停止进程；

说明：
- 此示例基于 protobuf channel local 后端示例，通过 **opentelemetry_plugin** 中的 `otp_metrics` 类型 filter 上报 channel metrics 数据；
