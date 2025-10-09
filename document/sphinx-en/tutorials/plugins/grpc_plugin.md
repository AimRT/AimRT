# GRPC Plugin

## Related Links

Reference example:
- {{ '[grpc_plugin]({}/src/examples/plugins/grpc_plugin)'.format(code_site_root_path_url) }}


## Plugin Overview

**grpc_plugin** is an RPC backend plugin implemented based on the [gRPC protocol](https://github.com/grpc/grpc/blob/master/doc/PROTOCOL-HTTP2.md). This plugin provides the following components for AimRT:
- `grpc` type RPC backend


The plugin configuration items are as follows:

| Node                      | Type          | Optional | Default   | Purpose |
| ----                      | ----          | ----     | ----      | ---- |
| thread_num                | unsigned int  | Optional | 2         | Number of threads used by the grpc plugin |
| listen_ip                 | string        | Optional | "0.0.0.0" | grpc listening IP |
| listen_port               | unsigned int  | Optional | -         | grpc listening port, the port must not be occupied |

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


Regarding the configuration of **grpc_plugin**, the usage notes are as follows:
- `thread_num` indicates the number of threads used by the grpc plugin.
- `listen_ip` is used to configure the address that the grpc service listens on. The default is "0.0.0.0". If you only want to listen on a specific network interface, you can change it to the specified IP.
- `listen_port` is used to configure the port that the grpc service listens on. This item is required. Users must ensure that the port is not occupied, otherwise the plugin will fail to initialize.
- When only using the client functionality, there is no need to configure the service listening address and port.

In addition, when using **grpc_plugin**, the RPC Server processing methods and RPC Client returns use the self-owned thread executor provided by **grpc_plugin**. When users block threads in callbacks, it may lead to exhaustion of the **grpc_plugin** thread pool, preventing further message reception/sending. As mentioned in the Module interface documentation, generally, if the task in the callback is very lightweight, it can be processed directly in the callback; but if the task in the callback is relatively heavy, it is best to schedule it to another dedicated executor for processing.

## grpc Type RPC Backend

The `grpc` type RPC backend is the RPC backend provided in **grpc_plugin**, used for calling and processing RPC requests via gRPC.

Notes:
* The current gRPC plugin only supports the HTTP2 plaintext protocol and does not support TLS encryption.
* The current gRPC plugin only supports unary RPC calls and does not support streaming RPC calls.
* The current gRPC plugin does not support HTTP2 message compression.


All configuration items for **grpc_plugin** are as follows:

| Node                          | Type      | Optional | Default | Purpose |
| ----                          | ----      | ----     | ----    | ---- |
| clients_options               | array     | Optional | []      | Rules for client-initiated RPC requests |
| clients_options[i].func_name  | string    | Required | ""      | RPC Func name, supports regular expressions |
| clients_options[i].server_url | string    | Required | ""      | URL requested when the RPC Func initiates a call |

Here is a simple client example:

```yaml
aimrt:
  plugin:
    plugins:
      - name: grpc_plugin
        path: ./libaimrt_grpc_plugin.so
        options:
          thread_num: 4
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


Here is a simple server example:

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


In the above examples, the Server listens on the local 50080 port, and the Client configures all RPC requests to be sent via the grpc backend to the address `http://127.0.0.1:50080`, which is the address the server listens on, thus completing the RPC call loop.

The [gRPC protocol](https://github.com/grpc/grpc/blob/master/doc/PROTOCOL-HTTP2.md) uses HTTP2 as the transport protocol, which is difficult to construct with conventional curl tools. You can use the [grpcurl tool](https://github.com/fullstorydev/grpcurl) to debug gRPC services. For example, with the following command, you can send a message to a specified module to trigger the corresponding callback:

```shell
grpcurl -plaintext \
    -import-path /path/to/aimrt/src/protocols/example \
    -proto rpc.proto \
    -d '{"msg": "test msg"}' \
    -max-time 1.0 \
    localhost:50050 aimrt.protocols.example.ExampleService/GetFooData \
    -vv
```


In the above command, `-plaintext` indicates the use of the h2c protocol (i.e., HTTP2 plaintext protocol without encryption), `-import-path` indicates the directory to import the gRPC protocol from, `-proto` indicates the file to import the gRPC protocol from, `-d` indicates the message content to send, `-max-time` indicates the maximum timeout, and `-vv` indicates verbose output.


Developers can also refer to the example in {{ '[grpc_plugin]({}/src/examples/plugins/grpc_plugin)'.format(code_site_root_path_url) }} to communicate with native grpc services.