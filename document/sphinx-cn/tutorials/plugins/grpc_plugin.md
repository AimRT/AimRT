# GRPC 插件

## 相关链接

参考示例：
- {{ '[grpc_plugin]({}/src/examples/plugins/grpc_plugin)'.format(code_site_root_path_url) }}


## 插件概述

**grpc_plugin** 是一个基于 [gRPC 协议](https://github.com/grpc/grpc/blob/master/doc/PROTOCOL-HTTP2.md) 实现的 rpc 后端插件，此插件为 AimRT 提供了以下组件：
- `grpc`类型 RPC 后端


插件的配置项如下：

| 节点                      | 类型          | 是否可选| 默认值 | 作用 |
| ----                      | ----          | ----  | ----      | ---- |
| thread_num                | unsigned int  | 可选  | 2         | grpc 插件使用的线程数 |
| listen_ip                 | string        | 可选  | "0.0.0.0" | grpc 监听 IP |
| listen_port               | unsigned int  | 必选  | -         | grpc 监听端口，端口不能被占用 |

以下是一个简单的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: grpc_plugin
        path: ./libaimrt_grpc_plugin.so
        options:
          thread_num: 4
          listen_ip: 127.0.0.1
          listen_port: 50082
```

关于 **grpc_plugin** 的配置，使用注意点如下：
- `thread_num` 表示 grpc 插件使用的线程数。
- `listen_ip` 用于配置 grpc 服务监听的地址，默认是"0.0.0.0"，如果仅想在指定网卡上监听，可以将其改为指定 IP。
- `listen_port` 用于配置 grpc 服务监听的端口，此项为必填项，使用者必须确保端口未被占用，否则插件会初始化失败。

此外，在使用 **grpc_plugin** 时，RPC Server 处理方法、RPC Client 返回时，使用的都是 **grpc_plugin** 提供的自有线程执行器，当使用者在回调中阻塞了线程时，有可能导致 **grpc_plugin** 线程池耗尽，从而无法继续接收/发送消息。正如 Module 接口文档中所述，一般来说，如果回调中的任务非常轻量，那就可以直接在回调里处理；但如果回调中的任务比较重，那最好调度到其他专门执行任务的执行器里处理。

## grpc 类型 RPC 后端

`grpc` 类型的 RPC 后端是 **grpc_plugin** 中提供的 RPC 后端，用于通过 gRPC 的方式来调用和处理 RPC 请求。

注意事项：
* 当前 gRPC 插件仅支持 HTTP2 明文协议，不支持 TLS 加密协议。
* 当前 gRPC 插件仅支持 protobuf 序列化协议，不支持其他序列化协议。
* 当前 gRPC 插件仅支持一元 RPC 调用，不支持流式 RPC 调用。
* 当前 gRPC 插件不支持 HTTP2 消息压缩。


**grpc_plugin** 所有的配置项如下：

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
      - name: grpc_plugin
        path: ./libaimrt_grpc_plugin.so
        options:
          thread_num: 4
          listen_ip: 127.0.0.1
          listen_port: 50081
  rpc:
    backends: 
      - type: grpc
        options:
          clients_options:
            - func_name: "(.*)"
              server_url: "http://127.0.0.1:50080"
    clients_options:
      - func_name: "(.*)"
        enable_backends: [grpc]
```

以下则是一个简单的服务端的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: grpc_plugin
        path: ./libaimrt_grpc_plugin.so
        options:
          thread_num: 4
          listen_ip: 127.0.0.1
          listen_port: 50080
  rpc:
    backends: 
      - type: grpc
    servers_options:
      - func_name: "(.*)"
        enable_backends: [grpc]
```

以上示例中，Server 端监听了本地的 50080 端口，Client 端则配置了所有的 RPC 请求都通过 grpc 后端请求到`http://127.0.0.1:50080`这个地址，也就是服务端监听的地址，从而完成 RPC 的调用闭环。

[gRPC 协议](https://github.com/grpc/grpc/blob/master/doc/PROTOCOL-HTTP2.md) 使用 HTTP2 作为传输协议，常规的 curl 工具较难构造，可以使用 [grpcurl 工具](https://github.com/fullstorydev/grpcurl) 来对 gRPC 服务进行调试，例如通过以下命令，即可发送一个消息到指定模块上去触发对应的回调：
```shell
grpcurl -plaintext \
    -import-path /path/to/aimrt/src/protocols/example \
    -proto rpc.proto \
    -d '{"msg": "test msg"}' \
    -max-time 1.0 \
    localhost:50050 aimrt.protocols.example.ExampleService/GetFooData \
    -vv
```

以上命令中，`-plaintext` 表示使用 h2c 协议（即 HTTP2 明文协议，不进行加密），`-import-path` 表示导入 gRPC 协议的目录，`-proto` 表示导入 gRPC 协议的文件，`-d` 表示发送的消息内容，`-max-time` 表示最大超时时间，`-vv` 表示输出详细信息。


开发者还可以参考{{ '[grpc_plugin]({}/src/examples/plugins/grpc_plugin)'.format(code_site_root_path_url) }}中的示例，与原生 grpc 服务进行通信。

