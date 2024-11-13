
# Mqtt 插件


## 相关链接

参考示例：
- {{ '[mqtt_plugin]({}/src/examples/plugins/mqtt_plugin)'.format(code_site_root_path_url) }}


## 插件概述

**mqtt_plugin**是一个基于 mqtt 协议实现的网络传输插件，此插件提供了以下组件：
- `mqtt`类型 RPC 后端
- `mqtt`类型 Channel 后端


插件的配置项如下：

| 节点                  | 类型   | 是否可选 | 默认值 | 作用                              |
| --------------------- | ------ | -------- | ------ | --------------------------------- |
| broker_addr           | string | 必选     | ""     | mqtt broker 的地址                |
| client_id             | string | 必选     | ""     | 本节点的 mqtt client id           |
| max_pkg_size_k        | int    | 可选     | 1024   | 最大包尺寸，单位：KB              |
| reconnect_interval_ms | int    | 可选     | 1000   | 重连 broker 的时间间隔， 单位：ms |
| truststore            | string | 可选     | ""     | CA证书路径                        |
| client_cert           | string | 可选     | ""     | 客户端证书路径                    |
| client_key            | string | 可选     | ""     | 客户端私钥路径                    |
| client_key_password   | string | 可选     | ""     | 客户端私钥设置的密码              |


关于**mqtt_plugin**的配置，使用注意点如下：
- `broker_addr`表示 mqtt broker 的地址，使用者必须保证有 mqtt 的 broker 运行在该地址，否则启动会失败。
- `client_id`表示本节点连接 mqtt broker 时的 client id。
- `max_pkg_size_k`表示传输数据时的最大包尺寸，默认 1 MB。注意，必须 broker 也要支持该尺寸才行。
- `reconnect_interval_ms`表示重连 broker 的时间间隔，默认 1 秒。
- `truststore`表示 broker 的 CA 证书路径，例如`/etc/emqx/certs/cacert.pem` 。当`broker_addr`的协议被配置为`ssl`或者`mqtts`时，该选项生效，用于指定 CA 证书路径，否则自动忽略该选项， 请注意若只配置该选项则视为单向认证。
- `client_cert`表示客户端证书路径，例如`/etc/emqx/certs/client-cert.pem`。当需要双向认证时使用，与`client_key`配合使用。如果 broker_addr 使用非加密协议，该选项将被忽略。
- `client_key`表示客户端私钥路径，例如`/etc/emqx/certs/client-key.pem`。当需要双向认证时使用，与`client_cert`配合使用。如果 broker_addr 使用非加密协议，该选项将被忽略。
-  `client_key_password`表示客户端私钥设置的密码，如果私钥设置了密码，则需要设置该选项。如果 broker_addr 使用非加密协议，该选项将被忽略。

**mqtt_plugin**插件基于[paho.mqtt.c](https://github.com/eclipse/paho.mqtt.c)封装，在使用时，Channel 订阅回调、RPC Server 处理方法、RPC Client 返回时，使用的都是**paho.mqtt.c**提供的线程，当使用者在回调中阻塞了线程时，有可能导致无法继续接收/发送消息。正如 Module 接口文档中所述，一般来说，如果回调中的任务非常轻量，那就可以直接在回调里处理；但如果回调中的任务比较重，那最好调度到其他专门执行任务的执行器里处理。


以下是一个简单的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: mqtt_plugin
        path: ./libaimrt_mqtt_plugin.so
        options:
          broker_addr: tcp://127.0.0.1:1883
          client_id: example_mqtt_client
          max_pkg_size_k: 1024
```


## mqtt 类型 RPC 后端


`mqtt`类型的 RPC 后端是**mqtt_plugin**中提供的一种 RPC 后端，用于通过 mqtt 的方式来调用和处理 AimRT RPC 请求。其所有的配置项如下：


| 节点                              | 类型   | 是否可选 | 默认值 | 作用                                                                         |
| --------------------------------- | ------ | -------- | ------ | ---------------------------------------------------------------------------- |
| timeout_executor                  | string | 可选     | ""     | Client 端发起 RPC 超时情况下的执行器                                         |
| clients_options                   | array  | 可选     | []     | Client 端发起 RPC 请求时的规则                                               |
| clients_options[i].func_name      | string | 必选     | ""     | RPC Func 名称，支持正则表达式                                                |
| clients_options[i].server_mqtt_id | string | 可选     | ""     | RPC Func 发起调用时请求的 mqtt 服务端 id                                     |
| clients_options[i].qos            | int    | 可选     | 2      | RPC Client 端 mqtt qos，取值范围：0/1/2                                      |
| servers_options                   | array  | 可选     | []     | 服务端处理 RPC 请求时的规则                                                  |
| servers_options[i].func_name      | string | 必选     | ""     | RPC Func 名称，支持正则表达式                                                |
| servers_options[i].allow_share    | bool   | 可选     | true   | 该 RPC 服务是否允许共享订阅，不允许的话该服务只能通过指定 server id 进行调用 |
| servers_options[i].qos            | int    | 可选     | 2      | RPC Server 端 mqtt qos，取值范围：0/1/2                                      |

以下是一个简单的客户端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: mqtt_plugin
        path: ./libaimrt_mqtt_plugin.so
        options:
          broker_addr: tcp://127.0.0.1:1883
          client_id: example_client
          max_pkg_size_k: 1024
  executor:
    executors:
      - name: timeout_handle
        type: time_wheel
  rpc:
    backends:
      - type: mqtt
        options:
          timeout_executor: timeout_handle
          clients_options:
            - func_name: "(.*)"
              qos: 0
    clients_options:
      - func_name: "(.*)"
        enable_backends: [mqtt]
```

以下则是一个简单的服务端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: mqtt_plugin
        path: ./libaimrt_mqtt_plugin.so
        options:
          broker_addr: tcp://127.0.0.1:1883
          client_id: example_server
          max_pkg_size_k: 1024
  rpc:
    backends:
      - type: mqtt
        options:
          servers_options:
            - func_name: "(.*)"
              allow_share: true
              qos: 0
    servers_options:
      - func_name: "(.*)"
        enable_backends: [mqtt]
```

以上示例中，Client 端和 Server 端都连上了`tcp://127.0.0.1:1883`这个地址的一个 Mqtt broker，Client 端也配置了所有的 RPC 请求都通过 mqtt 后端进行处理，从而完成 RPC 的调用闭环。

如果有多个 server 端同时注册了某个 RPC 服务，那么 client 端会随机的挑选一个 server 端发送请求。如果想指定某个 server 端处理，可以在 client 端的 ctx 中按照如下方法设置 ToAddr：
```cpp
auto ctx_ptr = proxy->NewContextSharedPtr();
// mqtt://{{target server mqtt id}}
ctx_ptr->SetToAddr("mqtt://target_server_mqtt_id");

auto status = proxy->Foo(ctx_ptr, req, rsp);
```


在整个 RPC 过程中，底层使用的 Mqtt Topic 名称格式如下：
- Server 端
  - 订阅 Req 使用的 topic（两个都会订阅）：
    - `$share/aimrt/aimrt_rpc_req/${func_name}`
    - `aimrt_rpc_req/${server_id}/${func_name}`
  - 发布 Rsp 使用的 topic：`aimrt_rpc_rsp/${client_id}/${func_name}`
- Client 端
  - 发布 Req 使用的 topic（二选一）：
    - `aimrt_rpc_req/${func_name}`
    - `aimrt_rpc_req/${server_id}/${func_name}`
  - 订阅 Rsp 使用的 topic：`aimrt_rpc_rsp/${client_id}/${func_name}`

其中`${client_id}`、`${server_id}`是 Client 端和 Server 端需要保证在同一个 Mqtt broker 环境下全局唯一的一个值，一般使用在 Mqtt broker 处注册的 id。`${func_name}`是 url 编码后的 AimRT RPC 方法名称。Server 端订阅使用共享订阅，保证只有一个服务端处理请求。此项特性需要支持 Mqtt5.0 协议的 Broker。


例如，client 端向 Mqtt broker 注册的 id 为`example_client`，func 名称为`/aimrt.protocols.example.ExampleService/GetBarData`，则`${client_id}`值为`example_client`，`${func_name}`值为`%2Faimrt.protocols.example.ExampleService%2FGetBarData`。



Client -> Server 的 Mqtt 数据包格式整体分 5 段:
- 序列化类型，一般是`pb`或`json`
- client 端想要 server 端回复 rsp 的 mqtt topic 名称。client 端自己需要订阅这个 mqtt topic
- msg id，4 字节，server 端会原封不动的封装到 rsp 包里，供 client 端定位 rsp 对应哪个 req
- context 区
  - context 数量，1 字节，最大 255 个 context
  - context_1 key, 2 字节长度 + 数据区
  - context_2 key, 2 字节长度 + 数据区
  - ...
- msg 数据

```
| n(0~255) [1 byte] | content type [n byte]
| m(0~255) [1 byte] | rsp topic name [m byte]
| msg id [4 byte]
| context num [1 byte]
| context_1 key size [2 byte] | context_1 key data [key_1_size byte]
| context_1 val size [2 byte] | context_1 val data [val_1_size byte]
| context_2 key size [2 byte] | context_2 key data [key_2_size byte]
| context_2 val size [2 byte] | context_2 val data [val_2_size byte]
| ...
| msg data [remaining byte]
```

Server -> Client 的 Mqtt 数据包格式整体分 4 段:
- 序列化类型，一般是`pb`或`json`
- msg id，4 字节，req 中的 msg id
- status code，4 字节，框架错误码，如果这个部分不为零，则代表服务端发生了错误，数据段将没有内容
- msg 数据

```
| n(0~255) [1 byte] | content type [n byte]
| msg id [4 byte]
| status code [4 byte]
| msg data [remaining byte]
```


## mqtt 类型 Channel 后端


`mqtt`类型的 Channel 后端是**mqtt_plugin**中提供的一种 Channel 后端，用于通过 mqtt 的方式来发布和订阅消息。其所有的配置项如下：


| 节点                             | 类型   | 是否可选 | 默认值 | 作用                                   |
| -------------------------------- | ------ | -------- | ------ | -------------------------------------- |
| pub_topics_options               | array  | 可选     | []     | 发布 Topic 时的规则                    |
| pub_topics_options[i].topic_name | string | 必选     | ""     | Topic 名称，支持正则表达式             |
| pub_topics_options[i].qos        | int    | 必选     | 2      | Publish 端 mqtt qos，取值范围：0/1/2   |
| sub_topics_options               | array  | 可选     | []     | 发布 Topic 时的规则                    |
| sub_topics_options[i].topic_name | string | 必选     | ""     | Topic 名称，支持正则表达式             |
| sub_topics_options[i].qos        | int    | 必选     | 2      | Subscribe 端 mqtt qos，取值范围：0/1/2 |


以下是一个简单的发布端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: mqtt_plugin
        path: ./libaimrt_mqtt_plugin.so
        options:
          broker_addr: tcp://127.0.0.1:1883
          client_id: example_publisher
          max_pkg_size_k: 1024
  channel:
    backends:
      - type: mqtt
        options:
          pub_topics_options:
            - topic_name: "(.*)"
              qos: 2
    pub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [mqtt]
```

以下则是一个简单的订阅端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: mqtt_plugin
        path: ./libaimrt_mqtt_plugin.so
        options:
          broker_addr: tcp://127.0.0.1:1883
          client_id: example_subscriber
          max_pkg_size_k: 1024
  channel:
    backends:
      - type: mqtt
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [mqtt]
```

以上示例中，发布端和订阅端都连上了`tcp://127.0.0.1:1883`这个地址的一个 Mqtt broker，发布端也配置了所有的消息都通过 mqtt 后端进行处理，订阅端也配置了所有消息都可以从 mqtt 后端触发回调，从而打通消息发布订阅的链路。


在这个过程中，底层使用的 Mqtt Topic 名称格式为：`/channel/${topic_name}/${message_type}`。其中，`${topic_name}`为 AimRT 的 Topic 名称，`${message_type}`为 url 编码后的 AimRT 消息名称。

例如，AimRT Topic 名称为`test_topic`，消息类型为`pb:aimrt.protocols.example.ExampleEventMsg`，则最终 Mqtt 的 topic 名称为：`/channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsg`。


在 AimRT 发布端发布数据到订阅端这个链路上，Mqtt 数据包格式整体分 3 段：
- 序列化类型，一般是`pb`或`json`
- context 区
  - context 数量，1 字节，最大 255 个 context
  - context_1 key, 2 字节长度 + 数据区
  - context_2 key, 2 字节长度 + 数据区
  - ...
- 数据

```
| n(0~255) [1 byte] | content type [n byte]
| context num [1 byte]
| context_1 key size [2 byte] | context_1 key data [key_1_size byte]
| context_1 val size [2 byte] | context_1 val data [val_1_size byte]
| context_2 key size [2 byte] | context_2 key data [key_2_size byte]
| context_2 val size [2 byte] | context_2 val data [val_2_size byte]
| ...
| msg data [len - 1 - n byte] |
```
