# aimrt.rpc


## 配置项概述

`aimrt.rpc`配置项用于配置 RPC 功能。其中的细节配置项说明如下：


| 节点                                | 类型      | 是否可选| 默认值 | 作用 |
| ----                                | ----      | ----  | ----  | ---- |
| backends                            | array     | 可选  | []    | RPC 后端列表 |
| backends[i].type                    | string    | 必选  | ""    | RPC 后端类型 |
| backends[i].options                 | map       | 可选  | -     | 具体 RPC 后端的配置 |
| clients_options                     | array     | 可选  | ""    | RPC Client 配置 |
| clients_options[i].func_name        | string    | 必选  | ""    | RPC Client 名称，支持正则表达式 |
| clients_options[i].enable_backends  | string array | 必选  | [] | RPC Client 允许使用的 RPC 后端列表 |
| clients_options[i].enable_filters   | string array | 可选  | [] | RPC Client 端需要加载的框架侧过滤器列表 |
| servers_options                     | array     | 可选  | ""    | RPC Server 配置 |
| servers_options[i].func_name        | string    | 必选  | ""    | RPC Server 名称，支持正则表达式 |
| servers_options[i].enable_backends  | string array | 必选  | [] | RPC Server 允许使用的 RPC 后端列表 |
| servers_options[i].enable_filters   | string array | 可选  | [] | RPC Server 端需要加载的框架侧过滤器列表 |



`aimrt.rpc`的配置说明如下：
- `backends`是一个数组，用于配置各个 Rpc 后端。
  - `backends[i].type`是 Rpc 后端的类型。AimRT 官方提供了`local`后端，部分插件也提供了一些 Rpc 后端类型。
  - `backends[i].options`是 AimRT 传递给各个 Rpc 后端的初始化参数，这部分配置格式由各个 Rpc 后端类型定义，请参考对应 Rpc 后端类型的文档。
- `clients_options`和`servers_options`是一个规则列表，用于控制各个 RPC 方法在发起调用或处理调用时使用的 Rpc 后端规则，其中：
  - `func_name`表示本条规则的 RPC 方法名称，以正则表达式形式配置，如果 RPC 方法名称命中了该正则表达式，则会应用该条规则。
  - `enable_backends`是一个字符串数组，表示如果 RPC 方法名称命中了本条规则，则此数组就定义了该 RPC 方法能被处理的 RPC 后端。注意，该数组中出现的名称都必须要在`backends`中配置过。
  - `enable_filters`是一个字符串数组，表示需要注册的框架侧 RPC 过滤器列表，数组中的顺序为过滤器注册的顺序。一些插件会提供一些框架侧过滤器，用于在 RPC 调用时做一些前置/后置操作。
  - 采用由上往下的顺序检查命中的规则，当某个 RPC 方法命中某条规则后，则不会针对此 RPC 方法再检查后面的规则。


在 AimRT 中，RPC 的前端接口和后端实现是解耦的，当开发者使用接口发起一个 RPC 调用，最终是要 RPC 后端来执行真正的 RPC 调用操作。

当 Client 端接口层发起一个 RPC 请求后，AimRT 框架会根据以下规则，在多个 RPC 后端中选择一个进行实际的处理：
- AimRT 框架会先根据`clients_options`配置确定某个 RPC 方法能被处理的RPC后端列表。
- AimRT 框架会先解析传入的 Context 里 Meta 参数中的`AIMRT_RPC_CONTEXT_KEY_TO_ADDR`项，如果其中手动配置了形如`xxx://yyy,zzz`这样的URL，则会解析出`xxx`字符串，并寻找同名的 RPC 后端进行处理。
- 如果没有配置 Context 参数，则根据该 RPC 方法能被处理的 RPC 后端列表顺序，依次尝试进行处理，直到遇到第一个真正进行处理的后端。

Server 端相对来说规则就比较简单，会根据`servers_options`的配置，接收并处理其中各个 RPC 后端传递过来的请求。



以下是一个简单的示例：
```yaml
aimrt:
  rpc:
    backends:
      - type: local
      - type: mqtt
    clients_options:
      - func_name: "(.*)"
        enable_backends: [local]
        enable_filters: []
    servers_options:
      - func_name: "(.*)"
        enable_backends: [local]
        enable_filters: []
```


## local 类型 Rpc 后端


`local`类型的 Rpc 后端是 AimRT 官方提供的一种 Rpc 后端，用于请求同进程中的其他模块提供的 RPC，它会自动判断 Client 端和 Server 端是否在同一个`Pkg`内，从而采用各种方式进行性能的优化。其所有的配置项如下：


| 节点                          | 类型      | 是否可选| 默认值 | 作用 |
| ----                          | ----      | ----  | ----  | ---- |
| timeout_executor              | string    | 可选  | ""    | Client端RPC超时情况下的执行器 |


使用注意点如下：
- Server 的执行器将使用 Client 调用时的执行器。同样，Client 调用结束后的执行器将使用 Server 返回时的执行器。
- 如果 Client 和 Server 在一个 Pkg 中，那么 Req、Rsp 的传递将直接通过指针来进行；如果 Client 和 Server 在一个 AimRT 进程中，但在不同的 Pkg 里，那么 Req、Rsp 将会进行一次序列化/反序列化再进行传递。
- Timeout 功能仅在 Client 和 Server 位于不同 Pkg 时生效。如果 Client 和 Server 在一个 Pkg 中，那么会为了性能优化直接传递 Req、Rsp 的指针，这种情况下为了保证 Client 端 Req、Rsp 的生命周期能覆盖 Server 端 Req、Rsp 的生命周期，Timeout 功能将不会生效。



以下是一个简单的示例：
```yaml
aimrt:
  executor:
    executors:
      - name: timeout_handle
        type: time_wheel
  rpc:
    backends:
      - type: local
        options:
          timeout_executor: timeout_handle
    clients_options:
      - func_name: "(.*)"
        enable_backends: [local]
    servers_options:
      - func_name: "(.*)"
        enable_backends: [local]
```
