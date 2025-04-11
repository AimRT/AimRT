

# Plugin Usage Examples

AimRT provides the following plugin usage examples:

- [plugins examples]({}/src/examples/plugins)
  - [grpc_plugin]({}/src/examples/plugins/grpc_plugin)
  - [iceoryx_plugin]({}/src/examples/plugins/iceoryx_plugin)
  - [log_control_plugin]({}/src/examples/plugins/log_control_plugin)
  - [mqtt_plugin]({}/src/examples/plugins/mqtt_plugin)
  - [net_plugin]({}/src/examples/plugins/net_plugin)
  - [opentelemetry_plugin]({}/src/examples/plugins/opentelemetry_plugin)
  - [parameter_plugin]({}/src/examples/plugins/parameter_plugin)
  - [record_playback_plugin]({}/src/examples/plugins/record_playback_plugin)
  - [ros2_plugin]({}/src/examples/plugins/ros2_plugin)
  - [time_manipulator_plugin]({}/src/examples/plugins/time_manipulator_plugin)
  - [zenoh_plugin]({}/src/examples/plugins/zenoh_plugin)
  - [topic_logger_plugin]({}/src/examples/plugins/topic_logger_plugin)
  - [echo_plugin]({}/src/examples/plugins/echo_plugin)
  - [proxy_plugin]({}/src/examples/plugins/proxy_plugin)

Notes about these examples:
- Each example has its own independent readme documentation. For details, please click the example links to view them.
- Most plugin examples are based on CPP interface demonstrations. Users need to enable the `AIMRT_BUILD_EXAMPLES` option when compiling AimRT. After compilation completes, these examples can be executed in the build directory.