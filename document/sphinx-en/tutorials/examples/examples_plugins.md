# Plugin Usage Examples

AimRT provides the following plugin usage examples:

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
  - {{ '[topic_logger_plugin]({}/src/examples/plugins/topic_logger_plugin)'.format(code_site_root_path_url) }}
  - {{ '[echo_plugin]({}/src/examples/plugins/echo_plugin)'.format(code_site_root_path_url) }}
  - {{ '[proxy_plugin]({}/src/examples/plugins/proxy_plugin)'.format(code_site_root_path_url) }}

Notes on these examples:
- Each example has its own independent readme document; for details, click the example link to view;
- Most plugin examples are based on CPP interface examples. Users need to enable the `AIMRT_BUILD_EXAMPLES` option when compiling AimRT. After compilation, these examples can be run in the build directory;