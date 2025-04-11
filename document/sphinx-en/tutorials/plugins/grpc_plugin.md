

# GRPC Plugin

## Related Links

Reference examples:
- {{ '[grpc_plugin]({}/src/examples/plugins/grpc_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**grpc_plugin** is an RPC backend plugin implemented based on the [gRPC protocol](https://github.com/grpc/grpc/blob/master/doc/PROTOCOL-HTTP2.md). This plugin provides the following components for AimRT:
- `grpc` type RPC backend

The configuration items of the plugin are as follows:

| Node                      | Type          | Optional | Default     | Purpose |
| ----                      | ----          | ----     | ----        | ----    |
| thread_num                | unsigned int  | Yes      | 2           | Number of threads used by the grpc plugin |
| listen_ip                 | string        | Yes      | "0.0.0.0"   | grpc listening IP |
| listen_port               | unsigned int  | No       | -           | grpc listening port, must be available |

Here is a simple example:  
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

Regarding the configuration of **grpc_plugin**, the following points should be noted:
- `thread_num` indicates the number of threads used by the grpc plugin.
- `listen_ip` configures the listening address of the grpc service, defaulting to "0.0.0.0". To listen on a specific network interface, modify it to a specific IP.
- `listen_port` configures the listening port of the grpc service. This is a mandatory item. Users must ensure the port is available, otherwise plugin initialization will fail.

Additionally, when using **grpc_plugin**, both the RPC Server processing methods and RPC Client returns use the plugin's own thread executor. If users block threads in callbacks, it may exhaust the **grpc_plugin** thread pool and prevent further message reception/sending. As mentioned in the Module interface documentation: generally, if tasks in callbacks are lightweight, they can be processed directly in the callback; but if tasks are heavy, it's better to schedule them to dedicated executors.

## grpc Type RPC Backend

The `grpc` type RPC backend provided by **grpc_plugin** is used for making and handling RPC requests through gRPC.

Notes:
* Current gRPC plugin only supports HTTP2 plaintext protocol, not TLS encryption.
* Current gRPC plugin only supports unary RPC calls, not streaming RPC.
* Current gRPC plugin does not support HTTP2 message compression.

Complete configuration items for **grpc_plugin**:

| Node                          | Type      | Optional | Default | Purpose |
| ----                          | ----      | ----     | ----    | ----    |
| clients_options               | array     | Yes      | []      | Rules for client RPC requests |
| clients_options[i].func_name  | string    | No       | ""      | RPC Func name supporting regular expressions |
| clients_options[i].server_url | string    | No       | ""      | Target URL for RPC Func calls |

Simple client example:  
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

Simple server example:  
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

In these examples: The Server listens on local port 50080, while the Client configures all RPC requests to be sent via grpc backend to `http://127.0.0.1:50080` (the server's listening address), completing the RPC call loop.

The [gRPC protocol](https://github.com/grpc/grpc/blob/master/doc/PROTOCOL-HTTP2.md) uses HTTP2 as transport protocol. Regular curl tools have difficulty constructing such requests. You can use the [grpcurl tool](https://github.com/fullstorydev/grpcurl) for gRPC service debugging. For example, use this command to send a message to a specific module and trigger corresponding callbacks:  
```shell
grpcurl -plaintext \
    -import-path /path/to/aimrt/src/protocols/example \
    -proto rpc.proto \
    -d '{"msg": "test msg"}' \
    -max-time 1.0 \
    localhost:50050 aimrt.protocols.example.ExampleService/GetFooData \
    -vv
```

In this command:  
- `-plaintext` indicates using h2c protocol (HTTP2 plaintext without encryption)  
- `-import-path` specifies gRPC protocol directory  
- `-proto` specifies gRPC protocol file  
- `-d` contains sent message content  
- `-max-time` sets maximum timeout  
- `-vv` enables verbose output

Developers can also refer to {{ '[grpc_plugin]({}/src/examples/plugins/grpc_plugin)'.format(code_site_root_path_url) }} examples to communicate with native grpc services.