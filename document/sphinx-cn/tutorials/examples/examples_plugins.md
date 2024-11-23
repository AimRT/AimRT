# 插件使用示例

AimRT 提供了以下插件使用示例：

- {{ '[plugins examples]({}/src/examples/plugins)'.format(code_site_root_path_url) }}
  - {{ '[grpc_plugin]({}/src/examples/plugins/grpc_plugin)'.format(code_site_root_path_url) }}
  - {{ '[iceoryx_plugin]({}/src/examples/plugins/iceoryx_plugin)'.format(code_site_root_path_url) }}
  - {{ '[log_control_plugin]({}/src/examples/plugins/log_control_plugin)'.format(code_site_root_path_url) }}
  - {{ '[mqtt_plugin]({}/src/examples/plugins/mqtt_plugin)'.format(code_site_root_path_url) }}
  - {{ '[net_plugin]({}/src/examples/plugins/net_plugin)'.format(code_site_root_path_url) }}
  - {{ '[opentelemetry_plugin]({}/src/examples/plugins/opentelemetry_plugin)'.format(code_site_root_path_url) }}
  - {{ '[parameter_plugin]({}/src/examples/plugins/parameter_plugin)'.format(code_site_root_path_url) }}
  - {{ '[record_playback_plugin]({}/src/examples/plugins/record_playback_plugin)'.format(code_site_root_path_url) }}
  - {{ '[ros2_plugin]({}/src/examples/plugins/ros2_plugin)'.format(code_site_root_path_url) }}
  - {{ '[time_manipulator_plugin]({}/src/examples/plugins/time_manipulator_plugin)'.format(code_site_root_path_url) }}
  - {{ '[zenoh_plugin]({}/src/examples/plugins/zenoh_plugin)'.format(code_site_root_path_url) }}
  - {{ '[echo_plugin]({}/src/examples/plugins/echo_plugin)'.format(code_site_root_path_url) }}
  - {{ '[proxy_plugin]({}/src/examples/plugins/proxy_plugin)'.format(code_site_root_path_url) }}

关于这些示例的说明：
- 每个示例都有自己独立的 readme 文档，详情请点击示例链接进入后查看；
- 大部分插件的示例都基于 CPP 接口示例，使用者需要在编译 AimRT 时开启 `AIMRT_BUILD_EXAMPLES` 选项，编译完成后即可在 build 目录下运行这些示例；

