

# echo Plugin

## Related Links

Reference Examples:
- {{ '[echo_plugin]({}/src/examples/plugins/echo_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**echo_plugin** is used for echoing messages in Channels. The plugin supports independent type_support_pkg configurations and allows specifying executors. The log_lvl must be set to Trace, Debug, or Info for proper operation.

Plugin configuration items:

| Node                              | Type          | Optional | Default  | Description |
| ----                              | ----          | ----     | ----     | ----        |
| type_support_pkgs                 | array         | Required | []       | Type support package configuration |
| type_support_pkgs[i].path         | string        | Required | ""       | Path to type support package |
| topic_meta_list                   | array         | Required | []       | Topics and types to echo |
| topic_meta_list[j].topic_name     | string        | Required | ""       | Target topic name |
| topic_meta_list[j].msg_type       | string        | Required | ""       | Message type to echo |
| topic_meta_list[j].echo_type      | string        | Optional | "json"   | Echo format. ROS2 supports "json"/"yaml", while PB only supports "json" |


### Simple Configuration Examples

For message echo formats: ROS2 message types support "json" and "yaml", while PB only supports "json".

Example configuration for PB message type with JSON format:
```yaml
aimrt:
  plugin:
    plugins:
      - name: echo_plugin
        path: ./libaimrt_echo_plugin.so
        options:
          type_support_pkgs:
            - path: ./libexample_event_ts_pkg.so
          topic_meta_list:
            - topic_name: test_topic
              msg_type: pb:aimrt.protocols.example.ExampleEventMsg
              echo_type: json
  log:
    core_lvl: Info # Trace/Debug/Info
    backends:
      - type: console
  channel:
    # ...
```

Example configuration for ROS2 message type with YAML format:
```yaml
aimrt:
  plugin:
    plugins:
      - name: echo_plugin
        path: ./libaimrt_echo_plugin.so
        options:
          type_support_pkgs:
            - path: ./libexample_event_ts_pkg.so
          topic_meta_list:
            - topic_name: test_topic
              msg_type: ros2:example_ros2/msg/RosTestMsg
              echo_type: yaml
  log:
    core_lvl: Info # Trace/Debug/Info
    backends:
      - type: console
  channel:
    # ...
```