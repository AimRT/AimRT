
# Net 插件

## 相关链接

参考示例：
- {{ '[net_plugin]({}/src/examples/plugins/net_plugin)'.format(code_site_root_path_url) }}


## 插件概述

**net_plugin**是一个基于 boost asio/beast 库实现的网络传输插件，此插件提供了以下组件：
- `http`类型 RPC 后端
- `http`类型 Channel 后端
- `tcp`类型 Channel 后端
- `udp`类型 Channel 后端

插件的配置项如下：

| 节点                      | 类型      | 是否可选| 默认值 | 作用 |
| ----                      | ----      | ----  | ----      | ---- |
| thread_num                | int       | 必选  | 2         | net 插件需要使用的线程数 |
| http_options              | map       | 可选  | -         | http 相关选项 |
| http_options.listen_ip    | string    | 可选  | "0.0.0.0" | http 监听 IP |
| http_options.listen_port  | int       | 必选  | -         | http 监听端口，端口不能被占用 |
| tcp_options               | map       | 可选  | -         | tcp 相关选项 |
| tcp_options.listen_ip     | string    | 可选  | "0.0.0.0" | tcp 监听 IP |
| tcp_options.listen_port   | int       | 必选  | -         | tcp 监听端口，端口不能被占用 |
| udp_options               | map       | 可选  | -         | udp 相关选项 |
| udp_options.listen_ip     | string    | 可选  | "0.0.0.0" | udp 监听 IP |
| udp_options.listen_port   | int       | 必选  | -         | udp 监听端口，端口不能被占用 |
| udp_options.max_pkg_size  | int       | 可选  | 1024      | udp 包最大尺寸，理论上最大不能超过 65515 |



关于**net_plugin**的配置，使用注意点如下：
- `thread_num`表示 net 插件使用的线程数。
- `http_options`是可选的，但只有当该节点被配置时，才能开启与 http 相关的功能，如 http Channel 后端、http RPC 后端。
  - `http_options.listen_ip`用于配置 http 服务监听的地址，默认是"0.0.0.0"，如果仅想在指定网卡上监听，可以将其改为指定 IP。
  - `http_options.listen_port`用于配置 http 服务监听的端口，此项为必填项，使用者必须确保端口未被占用，否则插件会初始化失败。
- `tcp_options`是可选的，但只有当该节点被配置时，才能开启与 tcp 相关的功能，如 tcp Channel 后端。
  - `tcp_options.listen_ip`用于配置 tcp 服务监听的地址，默认是"0.0.0.0"，如果仅想在指定网卡上监听，可以将其改为指定 IP。
  - `tcp_options.listen_port`用于配置 tcp 服务监听的端口，此项为必填项，使用者必须确保端口未被占用，否则插件会初始化失败。
- `udp_options`是可选的，但只有当该节点被配置时，才能开启与 udp 相关的功能，如 udp Channel 后端。
  - `udp_options.listen_ip`用于配置 udp 服务监听的地址，默认是"0.0.0.0"，如果仅想在指定网卡上监听，可以将其改为指定 IP。
  - `udp_options.listen_port`用于配置 udp 服务监听的端口，此项为必填项，使用者必须确保端口未被占用，否则插件会初始化失败。
  - `udp_options.max_pkg_size`用于配置 udp 包最大尺寸，理论上最大不能超过 65515。发布的消息序列化之后必须小于这个值，否则会发布失败。


此外，在使用**net_plugin**时，Channel 订阅回调、RPC Server 处理回调、RPC Client 返回时，使用的都是**net_plugin**提供的自有线程执行器，当使用者在回调中阻塞了线程时，有可能导致**net_plugin**线程池耗尽，从而无法继续接收/发送消息。正如 Module 接口文档中所述，一般来说，如果回调中的任务非常轻量，那就可以直接在回调里处理；但如果回调中的任务比较重，那最好调度到其他专门执行任务的执行器里处理。


以下是一个简单的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
          http_options:
            listen_ip: 127.0.0.1
            listen_port: 50080
          tcp_options:
            listen_ip: 127.0.0.1
            listen_port: 50081
          udp_options:
            listen_ip: 127.0.0.1
            listen_port: 50082
            max_pkg_size: 1024
```


## http 类型 RPC 后端


`http`类型的 RPC 后端是**net_plugin**中提供的一种 RPC 后端，用于通过 HTTP 的方式来调用和处理 RPC 请求。其所有的配置项如下：


| 节点                          | 类型      | 是否可选| 默认值 | 作用 |
| ----                          | ----      | ----  | ----  | ---- |
| clients_options               | array     | 可选  | []    | 客户端发起 RPC 请求时的规则 |
| clients_options[i].func_name  | string    | 必选  | ""    | RPC Func 名称，支持正则表达式 |
| clients_options[i].server_url | string    | 必选  | ""    | RPC Func 发起调用时请求的 url |


以下是一个简单的客户端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
          http_options:
            listen_ip: 127.0.0.1
            listen_port: 50081
  rpc:
    backends:
      - type: http
        options:
          clients_options:
            - func_name: "(.*)"
              server_url: http://127.0.0.1:50080
    clients_options:
      - func_name: "(.*)"
        enable_backends: [http]
```

以下则是一个简单的服务端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
          http_options:
            listen_ip: 127.0.0.1
            listen_port: 50080
  rpc:
    backends:
      - type: http
    servers_options:
      - func_name: "(.*)"
        enable_backends: [http]
```


以上示例中，Server 端监听了本地的 50080 端口，Client 端则配置了所有的 RPC 请求都通过 http 后端请求到`http://127.0.0.1:50080`这个地址，也就是服务端监听的地址，从而完成 RPC 的调用闭环。


Client 端向 Server 端发起调用时，遵循的格式如下：
- 使用 HTTP POST 方式发送，Body 中填充消息序列化后的请求包数据或回包数据；
- 由`content-type` Header 来定义 Body 内容的序列化方式，例如：
  - `content-type:application/json`
  - `content-type:application/protobuf`
- URL 的编码方式：`http://{IP:PORT}/rpc/{FUNC_NAME}`：
  - `{IP:PORT}`：对端的网络地址；
  - `{FUNC_NAME}`：URL 编码后的 RPC func 名称，以注册 RPC 时传入的为准；


只要遵循这个格式，使用者可以基于 PostMan 或者 Curl 等工具发起调用，以 JSON 作为序列化方式来提高可读性，例如通过以下命令，即可发送一个消息到指定模块上去触发对应的回调：
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.example.ExampleService/GetFooData' \
    -d '{"msg": "test msg"}'
```

基于这个特性，`http`类型的 RPC 后端常用于调试开发或测试阶段，可以通过辅助工具快速触发某个 Server 处理函数，或通过抓包工具来检查上下游之间的通信内容。


## http 类型 Channel 后端


`http`类型的 Channel 后端是**net_plugin**中提供的一种 Channel 后端，用于将消息通过 HTTP 的方式来发布和订阅消息。其所有的配置项如下：


| 节点                                  | 类型          | 是否可选| 默认值 | 作用 |
| ----                                  | ----          | ----  | ----  | ---- |
| pub_topics_options                    | array         | 可选  | []    | 发布 Topic 时的规则 |
| pub_topics_options[i].topic_name      | string        | 必选  | ""    | Topic 名称，支持正则表达式 |
| pub_topics_options[i].server_url_list | string array  | 必选  | []    | Topic 需要被发送的 url 列表 |

以下是一个简单的发布端的示例：
```yaml
aimrt:
  plugin:
    plugins:
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
        options:
          pub_topics_options:
            - topic_name: "(.*)"
              server_url_list: ["127.0.0.1:50080"]
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [http]
```

以下则是一个简单的订阅端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin 
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
          http_options:
            listen_ip: 127.0.0.1
            listen_port: 50080
  channel:
    backends:
      - type: http
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [http]
```

以上示例中，订阅端监听了本地的 50080 端口，发布端则配置了所有的 Topic 消息都通过 http 后端请求到`127.0.0.1:50080`这个地址，也就是服务端监听的地址，订阅端也配置了所有消息都可以从 http 后端触发回调，从而打通消息发布订阅的链路。

发布端向订阅端发布消息时，遵循的格式如下：
- 使用 HTTP POST 方式发送，Body 中填充消息序列化后的数据；
- 由`content-type` Header 来定义 Body 内容的序列化方式，例如：
  - `content-type:application/json`
  - `content-type:application/protobuf`
- URL的编码方式：`http://{IP:PORT}/channel/{TOPIC_NAME}/{MSG_TYPE}`：
  - `{IP:PORT}`：对端的网络地址；
  - `{TOPIC_NAME}`：URL 编码后的 Topic 名称；
  - `{MSG_TYPE}`：URL 编码后的消息类型名称，消息类型名称以 TypeSupport 中定义的为准；


只要遵循这个格式，使用者可以基于 PostMan 或者 Curl 等工具发起调用，以 JSON 作为序列化方式来提高可读性，例如通过以下命令，即可发送一个消息到指定模块上去触发对应的回调：
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsg' \
    -d '{"msg": "test msg", "num": 123}'
```

基于这个特性，`http`类型的 Channel 后端常用于调试开发或测试阶段，可以通过辅助工具快速触发某个回调，或通过抓包工具来检查上下游之间的通信内容。



## tcp 类型 Channel后端


`tcp`类型的 Channel 后端是**net_plugin**中提供的一种 Channel 后端，用于将消息通过 tcp 的方式来发布和订阅消息。其所有的配置项如下：


| 节点                                  | 类型          | 是否可选| 默认值 | 作用 |
| ----                                  | ----          | ----  | ----  | ---- |
| pub_topics_options                    | array         | 可选  | []    | 发布 Topic 时的规则 |
| pub_topics_options[i].topic_name      | string        | 必选  | ""    | Topic 名称，支持正则表达式 |
| pub_topics_options[i].server_url_list | string array  | 必选  | []    | Topic 需要被发送的 url 列表 |

以下是一个简单的发布端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin 
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
          tcp_options:
            listen_ip: 127.0.0.1
            listen_port: 50081
  channel:
    backends:
      - type: tcp
        options:
          pub_topics_options:
            - topic_name: "(.*)"
              server_url_list: ["127.0.0.1:50080"]
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [tcp]
```

以下则是一个简单的订阅端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin 
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
          tcp_options:
            listen_ip: 127.0.0.1
            listen_port: 50080
  channel:
    backends:
      - type: tcp
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [tcp]
```

以上示例中，订阅端监听了本地的 50080 端口，发布端则配置了所有的 Topic 消息都通过 tcp 后端请求到`127.0.0.1:50080`这个地址，也就是服务端监听的地址，从而打通消息发布订阅的链路。

注意，使用 tcp 后端传输消息时，数据格式是私有的，但数据格式将保持版本兼容。



## udp 类型 Channel后端


`udp`类型的 Channel 后端是**net_plugin**中提供的一种 Channel 后端，用于将消息通过 udp 的方式来发布和订阅消息。其所有的配置项如下：


| 节点                                  | 类型          | 是否可选| 默认值 | 作用 |
| ----                                  | ----          | ----  | ----  | ---- |
| pub_topics_options                    | array         | 可选  | []    | 发布 Topic 时的规则 |
| pub_topics_options[i].topic_name      | string        | 必选  | ""    | Topic 名称，支持正则表达式 |
| pub_topics_options[i].server_url_list | string array  | 必选  | []    | Topic 需要被发送的 url 列表 |

以下是一个简单的发布端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin 
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
          udp_options:
            listen_ip: 127.0.0.1
            listen_port: 50081
  channel:
    backends:
      - type: udp
        options:
          pub_topics_options:
            - topic_name: "(.*)"
              server_url_list: ["127.0.0.1:50080"]
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [udp]
```

以下则是一个简单的订阅端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: net_plugin 
        path: ./libaimrt_net_plugin.so
        options:
          thread_num: 4
          udp_options:
            listen_ip: 127.0.0.1
            listen_port: 50080
  channel:
    backends:
      - type: udp
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [udp]
```

以上示例中，订阅端监听了本地的 50080 端口，发布端则配置了所有的 Topic 消息都通过udp后端请求到`127.0.0.1:50080`这个地址，也就是服务端监听的地址，从而打通消息发布订阅的链路。

注意，使用 udp 后端传输消息时，数据格式是私有的，但数据格式将保持版本兼容。
