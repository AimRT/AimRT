# echo_plugin

## Related Links

Reference example:
- {{ '[echo_plugin]({}/src/examples/plugins/echo_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**echo_plugin** is used to echo messages in a Channel. The plugin supports an independent type_support_pkg and allows specifying an executor. The log_lvl must be set to one of Trace, Debug, or Info for it to work properly.

The plugin configuration items are as follows:

| Node                              | Type          | Optional | Default | Purpose |
| ----                              | ----          | ----     | ----    | ---- |
| type_support_pkgs                 | array         | Required | []      | type support package configuration |
| type_support_pkgs[i].path         | string        | Required | ""      | path to the type support package |
| topic_meta_list                   | array         | Required | []      | topics and types to echo |
| topic_meta_list[j].topic_name     | string        | Required | ""      | topic to echo |
| topic_meta_list[j].msg_type       | string        | Required | ""      | message type to echo |
| topic_meta_list[j].echo_type      | string        | Optional | "json"  | format for echoed messages; ros2 supports "json", "yaml", pb supports only "json" |

### Simple Example Configuration for Echoing Messages

For the format of echoed messages, ros2 message types support "json", "yaml", while pb supports only "json".

Below is a simple example configuration for a pb message type with echoed message format set to json:

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


Below is a simple example configuration for a ros2 message type with echoed message format set to yaml:

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
