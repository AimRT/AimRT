

# aimrt.rpc

## Configuration Overview

The `aimrt.rpc` configuration section is used to configure RPC functionality. Detailed configuration items are as follows:

| Node                              | Type      | Optional | Default | Description |
| ----                              | ----      | ----     | ----    | ----        |
| backends                          | array     | Optional | []      | RPC backend list |
| backends[i].type                  | string    | Mandatory | ""     | RPC backend type |
| backends[i].options               | map       | Optional | -       | Configuration for specific RPC backend |
| clients_options                   | array     | Optional | ""      | RPC Client configuration |
| clients_options[i].func_name      | string    | Mandatory | ""     | RPC Client name (supports regex) |
| clients_options[i].enable_backends | string array | Mandatory | []  | Allowed RPC backends for this client |
| clients_options[i].enable_filters | string array | Optional | []   | Framework-side filters to load for client |
| servers_options                   | array     | Optional | ""      | RPC Server configuration |
| servers_options[i].func_name      | string    | Mandatory | ""     | RPC Server name (supports regex) |
| servers_options[i].enable_backends | string array | Mandatory | []  | Allowed RPC backends for this server |
| servers_options[i].enable_filters | string array | Optional | []   | Framework-side filters to load for server |

Configuration notes:
- `backends` array configures various RPC backends:
  - `backends[i].type` specifies RPC backend type. AimRT officially provides `local` backend, while plugins may offer additional types.
  - `backends[i].options` contains initialization parameters for specific backend (format defined by each backend type).
- `clients_options` and `servers_options` define rule lists for RPC method handling:
  - `func_name` uses regex pattern to match RPC method names.
  - `enable_backends` specifies allowed backends (must be configured in `backends`).
  - `enable_filters` lists framework-side filters in registration order.
  - Rules are checked top-down, first match applies.

In AimRT, RPC frontend interfaces are decoupled from backend implementations. When initiating an RPC call, the framework selects backend through these steps:
1. Determine allowed backends via `clients_options`
2. Check `AIMRT_RPC_CONTEXT_KEY_TO_ADDR` in Context Meta for manual backend specification (`xxx://` format)
3. If no manual specification, try allowed backends in order until first successful processing

Server-side processing simply follows `servers_options` configuration to handle requests from configured backends.

Example configuration:
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

## local-type RPC Backend

The `local` RPC backend is AimRT's official implementation for intra-process communication between modules. It automatically optimizes performance based on whether client and server reside in the same `Pkg`. Configuration items:

| Node                | Type      | Optional | Default | Description |
| ----                | ----      | ----     | ----    | ----        |
| timeout_executor    | string    | Optional | ""      | Executor for handling RPC timeouts on client side |

Key considerations:
- Server executor uses client's calling executor. Response handling uses server's return executor.
- Intra-Pkg communication passes Req/Rsp via pointers for optimization. Cross-Pkg communication requires serialization/deserialization.
- Timeout mechanism only works for cross-Pkg communication. Intra-Pkg calls disable timeout to ensure object lifecycle consistency.

Example configuration:
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