

# Proxy Plugin

## Related Links

Reference example:
- {{ '[proxy_plugin]({}/src/examples/plugins/proxy_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**proxy_plugin** is used for proxy forwarding messages in Channels. The plugin supports independent type_support_pkg and allows specifying executors (which need to be thread-safe). During usage, the plugin will register one or more Channel Subscribers or Publishers based on configurations.

The configuration items are as follows:

| Node                              | Type          | Optional | Default  | Description |
| ----                              | ----          | ----     | ----     | ----        |
| type_support_pkgs                 | array         | Required | []       | Type support package configuration |
| type_support_pkgs[i].path         | string        | Required | ""       | Path of type support package |
| proxy_actions                     | array         | Required | []       | Proxy forwarding configuration |
| proxy_actions[i].name            | string        | Required | ""       | Proxy action name |
| proxy_actions[i].options         | object        | Required | {}       | Proxy action options |
| proxy_actions[i].options.executor| string        | Required | ""       | Proxy executor |
| proxy_actions[i].options.topic_meta_list                   | array         | Required | []       | Target topics and types for proxy |
| proxy_actions[i].options.topic_meta_list[j].topic_name     | string        | Required | ""       | Source topic name |
| proxy_actions[i].options.topic_meta_list[j].msg_type       | string        | Required | ""       | Message type |
| proxy_actions[i].options.topic_meta_list[j].pub_topic_name | array         | Required | []       | Forwarded topic names |


### Basic Proxy Configuration Example

Below is an example configuration that proxies messages from an HTTP-backed topic to two Zenoh and ROS2-backed topics. For proxy_plugin, each action requires specifying an executor. In the Channel configuration, backend specifications are required for both subscribed topics and forwarded topics. For other related plugin configurations, please refer to [net_plugin](./net_plugin.md), [zenoh_plugin](./zenoh_plugin.md), and [ros2_plugin](./ros2_plugin.md).

```yaml
aimrt:
  plugin:
    plugins:
      - name: proxy_plugin
        path: ./libaimrt_proxy_plugin.so
        options:
          type_support_pkgs:
            - path: ./libexample_event_ts_pkg.so
          proxy_actions:
            - name: my_proxy
              options:
                executor: proxy_plugin_executor
                topic_meta_list:
                  - sub_topic_name: test_topic_http
                    pub_topic_name: [test_topic_zenoh, test_topic_ros2]
                    msg_type: pb:aimrt.protocols.example.ExampleEventMsg
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so
      - name: ros2_plugin
        path: ./libaimrt_ros2_plugin.so
        options:
          node_name: example_ros2_pb_chn_publisher_node
          executor_type: MultiThreaded # SingleThreaded/StaticSingleThreaded/MultiThreaded
          executor_thread_num: 2
      - name: net_plugin
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
          http_options:
          listen_ip: 127.0.0.1
          listen_port: 50081
  channel:
    backends:
      - type: http
      - type: zenoh
      - type: ros2
    sub_topics_options:
      - topic_name: test_topic_http
        enable_backends: [http]
    pub_topics_options:
      - topic_name: test_topic_zenoh
        enable_backends: [zenoh]
      - topic_name: test_topic_ros2
        enable_backends: [ros2]
    # ...
```