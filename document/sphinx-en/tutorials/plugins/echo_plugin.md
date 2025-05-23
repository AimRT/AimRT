# echo Plugin

## Related Links

Reference example:
- {{ '[echo_plugin]({}/src/examples/plugins/echo_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

The **echo_plugin** is used to echo messages in a Channel. The plugin supports independent type_support_pkg and allows specifying an executor. The log_lvl must be set to one of Trace, Debug, or Info for proper operation.

The plugin configuration items are as follows:

| Node                              | Type          | Optional | Default  | Purpose |
| ----                              | ----          | ----     | ----     | ----    |
| type_support_pkgs                 | array         | Required | []       | type support package configuration |
| type_support_pkgs[i].path         | string        | Required | ""       | Path of the type support package |
| topic_meta_list                   | array         | Required | []       | Topics and types to echo |
| topic_meta_list[j].topic_name     | string        | Required | ""       | Topic to echo |
| topic_meta_list[j].msg_type       | string        | Required | ""       | Message type to echo |
| topic_meta_list[j].echo_type      | string        | Optional | "json"   | Echo message format. For ROS2, supports "json" and "yaml". For PB, only supports "json". |

### Simple Example Configuration for Echo Messages

For echo message formats, ROS2 message types support "json" and "yaml", while PB only supports "json".

Here is a simple example configuration for PB message type with echo message format as json:
```yaml
aimrt:
  plugin:
    plugins:
      - name: echo_plugin
        path: ./libaimrt_echo_plugin.so
        options:
          type_support_pkgs:
            - path: ./libexample_pb_ts.so
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

Here is a simple example configuration for ROS2 message type with echo message format as yaml:
```yaml
aimrt:
  plugin:
    plugins:
      - name: echo_plugin
        path: ./libaimrt_echo_plugin.so
        options:
          type_support_pkgs:
            - path: ./libexample_ros2_ros2_ts.so
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