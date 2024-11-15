# Zenoh 插件


## 相关链接

参考示例：
- {{ '[zenoh_plugin]({}/src/examples/plugins/zenoh_plugin)'.format(code_site_root_path_url) }}


## 插件概述

**zenoh_plugin** 是一个轻量级的、高效的、实时的数据传输插件，它旨在为分布式系统提供低延迟、高吞吐量的数据传输和处理能力。当需要以下业务场景的时候，zenoh 插件将非常合适：
- `服务发现`机制的通信系统；
- 灵活的网络拓扑结构；
- 低延迟、高吞吐量的网络通信和数据传输；
- SHM 和 非 SHM 两种传输模式；
- 
此插件为 AimRT 提供以下组件：
- `zenoh` 类型 Rpc 后端
- `zenoh` 类型 Channel 后端


插件的配置项如下：

|        节点        |  类型  | 是否可选 | 默认值 |              作用               |
| :----------------: | :----: | :------: | :----: | :-----------------------------: |
|   shm_pool_size    |  int   |   可选   | 10 MB  |   共享内存池的大小 ， 单位：B   |
| shm_init_loan_size |  int   |   可选   |  1 KB  | 共享内存初始借用大小 ， 单位：B |
|  native_cfg_path   | string |   可选   |   ""   |   使用zenoh提供的原生配置文件   |
|    limit_domain    | string |   可选   |   ""   |     对插件的通信域进行限制      |


关于**zenoh_plugin**的配置，使用注意点如下：
- `shm_pool_size` 表示共享内存池的大小，单位：B，默认值是 10 MB，可以根据实际情况调整， 如果不使用共享内存，可忽略该配置项。 如果剩余共享内存不足以满足数据传输需求，则会自动切换到非共享内存传输模式 。
- `shm_init_loan_size` 表示向共享内存池初始借用大小，单位：B，默认值是 1 KB，可以根据实际情况调整， 如果不使用共享内存，可忽略该配置项。
- `native_cfg_path` 表示 zenoh 提供的原生配置文件的路径，可以通过配置该文件来灵活配置 zenoh 的网络结构。如果不填写则默认使用 zenoh 官方提供的默认配置，具体配置内容请参考 zenoh 官方关于[configuration](https://zenoh.io/docs/manual/configuration/)的说明，您也可以直接修改 {{ '[zenoh_native_config.json5]({}/src/examples/plugins/zenoh_plugin/install/linux/bin/cfg/zenoh_native_config.json5)'.format(code_site_root_path_url) }}文件来自定义配置，这里列举几个常用的配置项：

|            配置项             |                                 作用                                  | zenoh_native_config中的配置值 |
| :---------------------------: | :-------------------------------------------------------------------: | :---------------------------: |
|  scouting.multicast. enabled  |           是否开启多播，这将允许多个 zenoh 节点自动发现彼此           |             true              |
|  scouting.multicast. address  |                             配置多播地址                              |       224.0.0.224:7446        |
| scouting.multicast. interface |                          配置所用的网络接口                           |             auto              |
|       listen.endpoints        |                          需要主动监听的地址                           |               -               |
| transport.unicast.lowlatency  | 是否启动最低时延，若开启将有助于提升传输速率，注意其不能与qos同时开启 |             false             |
| transport.unicast.qos.enabled |           是否启动服务质量，注意其不能与lowlatency同时开启            |             true              |


- limit_domain 表示插件的通信域，其兼容 zenoh 强大的Key & Key Expression。如果不填写则使用插件的默认的通信域（即消息的 topic ），只有相`匹配`的域才能进行通信，具体的书写格式如下：

```shell

#请不要以"/"开始，中间以"/"分隔，结尾不要带"/" （与zenoh官方书写方式一致），如：

xxx/yyy/zzz

```
  最简单的一种匹配就是二者域相同，除此之外，zenoh 官方提供了更灵活的匹配机制，具体可查阅 zenoh 官方关于[key](https://zenoh.io/docs/manual/abstractions/)的解释。

以下是一个简单的示例：

```yaml
aimrt:
  plugin:
    plugins:
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so 
        options: 
          native_cfg_path: ./cfg/zenoh_native_config.json5       
```


## zenoh 类型 Rpc 后端 
`zenoh`类型的 Rpc后端是**zenoh_plugin**中提供的一种 Rpc 后端，主要用来构建请求-响应模型。其所有的配置项如下：

| 节点                           | 类型   | 是否可选 | 默认值 | 作用                                 |
| ------------------------------ | ------ | -------- | ------ | ------------------------------------ |
| timeout_executor               | string | 可选     | ""     | Client 端发起 RPC 超时情况下的执行器 |
| clients_options                | array  | 可选     | []     | Client 端发起 RPC 请求时的规则       |
| clients_options[i].func_name   | string | 必选     | ""     | RPC Func 名称，支持正则表达式        |
| clients_options[i].shm_enabled | bool   | 必选     | false  | RPC Func 是否使用共享内存通信        |
| servers_options                | array  | 可选     | []     | 服务端处理 RPC 请求时的规则          |
| servers_options[i].func_name   | string | 必选     | ""     | RPC Func 名称，支持正则表达式        |
| servers_options[i].shm_enabled | bool   | 必选     | false  | RPC Func 是否使用共享内存通信        |

注意： zenoh 支持 SHM 和 非 SHM 的自动转换， 即如果数据离开其所在的SHM 域，则自动切换到非 SHM 通信。 例如，如果节点 A 和 节点 B 都设置的共享内存，但其不再同一机器上，仍可以进行通信，因为数据会自动切换到非共享内存的传输模式。

以下是一个简单的客户端的示例：

```yaml
aimrt:
  plugin:
    plugins:
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so
        options: 
          native_cfg_path: ./cfg/zenoh_native_config.json5
          shm_pool_size: 10240
  executor:
    executors:
      - name: timeout_handle
        type: time_wheel
        options:
          bind_executor: work_thread_pool
  rpc:
    backends:
      - type: zenoh
        options: 
          timeout_executor: timeout_handle
          clients_options: 
            - func_name: "(.*)"
              shm_enabled: false
    clients_options:
      - func_name: "(.*)"
        enable_backends: [zenoh]

```

以下是一个简单的服务端的示例：

```yaml

aimrt:
  plugin:
    plugins:
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so
        options:
          native_cfg_path: ./cfg/zenoh_native_config.json5                 
  rpc:
    backends:
      - type: zenoh
        options: 
          timeout_executor: timeout_handle
          servers_options: 
            - func_name: "(.*)"
              shm_enabled: true
    servers_options:
      - func_name: "(.*)"
        enable_backends: [zenoh]

```
以上示例中，Client 端和 Server 端都采用服务发现机制，即在统一网络中的两个端点可自动发现彼此并建立连接。

在整个 RPC 过程中，底层使用的 Zenoh Topic 名称格式如下：
- Server 端
  - 订阅 Req 使用的 topic：
    - `req/aimrt_rpc/${func_name}/${limit_domain}>`
  - 发布 Rsp 使用的 topic：`rsp/aimrt_rpc/${func_name}/${limit_domain}`
- Client 端
  - 发布 Req 使用的 topic：
    - `req/aimrt_rpc/${func_name}/ ${limit_domain}`
  - 订阅 Rsp 使用的 topic：`rsp_/imrt_rpc/${func_name}/${limit_domain}`

`${func_name}`是 url 编码后的 AimRT RPC 方法名称。

 
例如，对于一个 client 的请求来说，若 func 名称为`/aimrt.protocols.example.ExampleService/GetBarData`, limit_domain 没有配置，则`最终的topic名称为`:req/aimrt_rpc/%2Faimrt.protocols.example.ExampleService%2FGetBarData`。



Client -> Server 的 Zenoh 数据包格式整体分 5 段:
- 序列化类型，一般是`pb`或`json`
- client 端想要 server 端回复 rsp 的 zenoh topic 名称。client 端自己需要订阅这个 zenoh topic
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

Server -> Client 的 Zenoh 数据包格式整体分 4 段:
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

## zenoh 类型 Channel 后端

`zenoh`类型的 Channel 后端是**zenoh_plugin**中提供的一种Channel后端，主要用来构建发布和订阅模型。其所有的配置项如下：

| 节点                              | 类型   | 是否可选 | 默认值 | 作用                            |
| --------------------------------- | ------ | -------- | ------ | ------------------------------- |
| pub_topics_options                | array  | 可选     | []     | 发布 Topic 时的规则             |
| pub_topics_options[i].topic_name  | string | 必选     | ""     | Topic 名称，支持正则表达式      |
| pub_topics_options[i].shm_enabled | bool   | 必选     | false  | Publish 端 是否使用共享内存通信 |

注意： zenoh 支持 SHM 和 非 SHM 的自动转换， 即如果数据离开其所在的SHM 域，则自动切换到非 SHM 通信。 例如，如果节点 A 和 节点 B 都设置的共享内存，但其不再同一机器上，仍可以进行通信，因为数据会自动切换到非共享内存的传输模式。

以下是一个简单的发布端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so
        options: 
          shm_pool_size: 1024      
  channel:
    backends:
      - type: zenoh
        options:
          pub_topics_options: 
            - topic_name: "(.*)"
              shm_enabled: false
    pub_topics_options:
      - topic_name: "(.*)" 
        enable_backends: [zenoh]
```


以下是一个简单的订阅端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: zenoh_plugin
        path: ./libaimrt_zenoh_plugin.so
        options: 
          native_cfg_path: ./cfg/zenoh_native_config.json5      
channel:
    backends:
      - type: zenoh
    sub_topics_options:
      - topic_name: "(.*)"
        enable_backends: [zenoh]
```

以上示例中都使用 zeonh 的服务发现机制，即在统一网络中的两个端点可自动发现彼此并建立连接。

在这个过程中，底层使用的 Topic 名称格式为：`channel/${topic_name}/${message_type}${limit_domain}`。其中，`${topic_name}`为 AimRT 的 Topic 名称，`${message_type}`为 url 编码后的 AimRT 消息名称， `${limit_domain}`为插件的限制域。这个 Topic 被设置成为 Zenoh 最终的键表达式（Keyxpr），这是 Zenoh 的提供的资源标识符，只有键表达式匹配的订阅者和发布者才能够进行通信。

例如，AimRT Topic 名称为`test_topic`，消息类型为`pb:aimrt.protocols.example.ExampleEventMsg`，限制域为`room1/A2`，则最终 Zenoh 的 topic 名称为：`channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsgroom1/A2`。如果订阅者和发布者的 Topic 均为`channel/test_topic/pb%3Aaimrt.protocols.example.ExampleEventMsgroom1/A2`，则二者可以进行通信。

在AimRT发布端发布数据到订阅端这个链路上，Zenoh 数据包格式整体分 3 段：
- 序列化类型，一般是`pb`或`json`
- context 区
  - context 数量，1 字节，最大 255 个 context
  - context_1 key, 2 字节长度 + 数据区
  - context_2 key, 2 字节长度 + 数据区
  - ...
- 数据

数据包格式如下：
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
