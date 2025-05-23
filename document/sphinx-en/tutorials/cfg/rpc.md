# aimrt.rpc

## Configuration Overview

The `aimrt.rpc` configuration is used to set up RPC functionality. The detailed configuration items are described below:

| Node                                | Type      | Optional | Default | Purpose |
| ----                                | ----      | ----     | ----    | ----    |
| backends                            | array     | Optional | []      | List of RPC backends |
| backends[i].type                    | string    | Required | ""      | Type of RPC backend |
| backends[i].options                 | map       | Optional | -       | Configuration for specific RPC backend |
| clients_options                     | array     | Optional | ""      | RPC Client configuration |
| clients_options[i].func_name        | string    | Required | ""      | RPC Client name (supports regular expressions) |
| clients_options[i].enable_backends  | string array | Required | [] | List of allowed RPC backends for the RPC Client |
| clients_options[i].enable_filters   | string array | Optional | [] | List of framework-side filters to load for RPC Client |
| servers_options                     | array     | Optional | ""      | RPC Server configuration |
| servers_options[i].func_name        | string    | Required | ""      | RPC Server name (supports regular expressions) |
| servers_options[i].enable_backends  | string array | Required | [] | List of allowed RPC backends for RPC Server |
| servers_options[i].enable_filters   | string array | Optional | [] | List of framework-side filters to load for RPC Server |

The `aimrt.rpc` configuration is explained as follows:
- `backends` is an array used to configure various RPC backends.
  - `backends[i].type` specifies the type of RPC backend. AimRT officially provides the `local` backend, while some plugins may offer additional RPC backend types.
  - `backends[i].options` contains initialization parameters passed by AimRT to each RPC backend. The format of this configuration is defined by each RPC backend type. Please refer to the documentation of the corresponding RPC backend type.
- `clients_options` and `servers_options` are rule lists that control which RPC backends are used when making or handling RPC calls:
  - `func_name` represents the RPC method name for this rule, configured as a regular expression. If an RPC method name matches this regular expression, the rule will be applied.
  - `enable_backends` is a string array indicating which RPC backends can handle the RPC method when it matches this rule. Note that all names in this array must be configured in `backends`.
  - `enable_filters` is a string array listing the framework-side RPC filters to be registered, with the array order representing the registration sequence. Some plugins provide framework-side filters for performing pre/post operations during RPC calls.
  - Rules are checked from top to bottom. Once an RPC method matches a rule, subsequent rules won't be checked for that method.

In AimRT, the RPC frontend interface is decoupled from the backend implementation. When a developer uses the interface to initiate an RPC call, it's ultimately processed by an RPC backend.

When the Client interface layer initiates an RPC request, the AimRT framework selects an RPC backend for actual processing based on the following rules:
- The framework first determines the list of RPC backends that can handle the method based on `clients_options` configuration.
- The framework then parses the `AIMRT_RPC_CONTEXT_KEY_TO_ADDR` item in the Meta parameters of the incoming Context. If a URL like `xxx://yyy,zzz` is manually configured, it extracts the `xxx` string and looks for an RPC backend with the same name.
- If no Context parameter is configured, the framework tries each RPC backend in the allowed list sequentially until finding the first one that can process the request.

The Server side has simpler rules, receiving and processing requests from various RPC backends according to the `servers_options` configuration.

Here's a simple example:
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

## Local Type RPC Backend

The `local` type RPC backend is an official RPC backend provided by AimRT, used for requesting RPCs from other modules within the same process. It automatically determines whether the Client and Server are in the same `Pkg` and optimizes performance accordingly. All its configuration items are as follows:

| Node                          | Type      | Optional | Default | Purpose |
| ----                          | ----      | ----     | ----    | ----    |
| timeout_executor              | string    | Optional | ""      | Executor for RPC timeout scenarios on Client side |

Usage notes:
- The Server's executor will use the same executor as the Client during invocation. Similarly, the executor after Client invocation will use the executor from Server response.
- If Client and Server are in the same Pkg, Req and Rsp are passed directly via pointers. If they're in the same AimRT process but different Pkgs, Req and Rsp will undergo serialization/deserialization before transfer.
- Timeout functionality only works when Client and Server are in different Pkgs. If they're in the same Pkg, pointers are passed directly for performance optimization, and timeout won't take effect to ensure the lifecycle of Client-side Req/Rsp covers Server-side Req/Rsp.

Here's a simple example:
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