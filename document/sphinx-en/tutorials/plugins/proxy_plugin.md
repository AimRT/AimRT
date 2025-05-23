# proxy Plugin

## Related Links

Reference example:
- {{ '[proxy_plugin]({}/src/examples/plugins/proxy_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**proxy_plugin** is used for proxying and forwarding messages in Channels. The plugin supports independent type_support_pkg and allows specifying executors (which need to be thread-safe). During usage, the plugin will register one or more Channel Subscribers or Publishers based on the configuration.

The configuration items for the plugin are as follows:

| Node                              | Type          | Optional | Default  | Purpose |
| ----                              | ----          | ----     | ----     | ----    |
| type_support_pkgs                 | array         | Required | []       | type support package configuration |
| type_support_pkgs[i].path         | string        | Required | ""       | Path of the type support package |
| proxy_actions                     | array         | Required | []       | Proxy forwarding configuration |
| proxy_actions[i].name             | string        | Required | ""       | Proxy forwarding name |
| proxy_actions[i].options          | object        | Required | {}       | Proxy forwarding configuration |
| proxy_actions[i].options.executor | string        | Required | ""       | Proxy forwarding executor |
| proxy_actions[i].options.topic_meta_list                   | array         | Required | []       | Topics and types to be proxied |
| proxy_actions[i].options.topic_meta_list[j].topic_name     | string        | Required | ""       | Topic to be proxied |
| proxy_actions[i].options.topic_meta_list[j].msg_type       | string        | Required | ""       | Message type to be proxied |
| proxy_actions[i].options.topic_meta_list[j].pub_topic_name | array         | Required | []       | Target topic after proxying |

Note that **proxy_plugin** manages proxy forwarding actions in `action` units. Each proxy `action` can have its own executor, topics, and other parameters. During usage, appropriate resources can be allocated for each action based on actual data size and frequency.

### Simple Example Configuration for Proxy Forwarding

The following is a simple example configuration that proxies messages from a topic with an http backend to two topics with zenoh and ros2 backends. For proxy_plugin, an executor needs to be specified for each action, and at the channel level, a backend needs to be specified for each subscribed topic and forwarded topic. For configurations of other related plugins, please refer to [net_plugin](./net_plugin.md), [zenoh_plugin](./zenoh_plugin.md), and [ros2_plugin](./ros2_plugin.md);

```yaml
aimrt:
  plugin:
    plugins:
      - name: proxy_plugin
        path: ./libaimrt_proxy_plugin.so
        options:
          type_support_pkgs:
            - path: ./libexample_pb_ts.so
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