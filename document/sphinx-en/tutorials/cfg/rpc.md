# aimrt.rpc


## Configuration Overview

The `aimrt.rpc` configuration item is used to configure RPC functionality. The detailed configuration items are described below:


| Node                                | Type      | Optional | Default | Purpose |
| ----                                | ----      | ----     | ----    | ---- |
| backends                            | array     | Optional | []      | List of RPC backends |
| backends[i].type                    | string    | Required | ""      | Type of RPC backend |
| backends[i].options                 | map       | Optional | -       | Configuration for specific RPC backend |
| clients_options                     | array     | Optional | ""      | RPC Client configuration |
| clients_options[i].func_name        | string    | Required | ""      | RPC Client name, supports regular expressions |
| clients_options[i].enable_backends  | string array | Required | [] | List of RPC backends allowed for this RPC Client |
| clients_options[i].enable_filters   | string array | Optional | [] | List of framework-side filters to load on RPC Client side |
| servers_options                     | array     | Optional | ""      | RPC Server configuration |
| servers_options[i].func_name        | string    | Required | ""      | RPC Server name, supports regular expressions |
| servers_options[i].enable_backends  | string array | Required | [] | List of RPC backends allowed for this RPC Server |
| servers_options[i].enable_filters   | string array | Optional | [] | List of framework-side filters to load on RPC Server side |



The configuration description for `aimrt.rpc` is as follows:
- `backends` is an array used to configure various Rpc backends.
  - `backends[i].type` is the type of Rpc backend. AimRT officially provides the `local` backend, and some plugins also provide certain Rpc backend types.
  - `backends[i].options` are initialization parameters passed by AimRT to each Rpc backend. The format of this configuration is defined by each Rpc backend type; please refer to the documentation for the corresponding Rpc backend type.
- `clients_options` and `servers_options` are rule lists used to control which Rpc backend rules are used when initiating or processing calls for each RPC method, where:
  - `func_name` indicates the RPC method name for this rule, configured as a regular expression. If the RPC method name matches this regular expression, this rule will be applied.
  - `enable_backends` is a string array indicating that if the RPC method name matches this rule, this array defines the RPC backends that can handle this RPC method. Note that all names appearing in this array must be configured in `backends`.
  - `enable_filters` is a string array indicating the list of framework-side RPC filters to register, with the order in the array being the order of filter registration. Some plugins provide framework-side filters for pre/post operations during RPC calls.
  - Rules are checked from top to bottom. When an RPC method matches a rule, subsequent rules will not be checked for this RPC method.


In AimRT, the RPC frontend interface and backend implementation are decoupled. When a developer initiates an RPC call using the interface, the actual RPC call operation is ultimately executed by the RPC backend.

When the Client-side interface layer initiates an RPC request, the AimRT framework will select one from multiple RPC backends for actual processing based on the following rules:
- The AimRT framework first determines the list of RPC backends that can handle a specific RPC method based on the `clients_options` configuration.
- The AimRT framework first parses the `AIMRT_RPC_CONTEXT_KEY_TO_ADDR` item in the Meta parameters of the incoming Context. If a URL in the format `xxx://yyy,zzz` is manually configured, it will parse the `xxx` string and look for an RPC backend with the same name for processing.
- If no Context parameter is configured, it will attempt processing in order according to the list of RPC backends that can handle this RPC method, until the first backend that actually processes it is encountered.

The Server side has relatively simpler rules. It will receive and process requests passed by each RPC backend according to the configuration in `servers_options`.



Here is a simple example:

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



## local Type Rpc Backend


The `local` type Rpc backend is an Rpc backend officially provided by AimRT, used to request RPCs provided by other modules in the same process. It automatically determines whether the Client and Server are within the same `Pkg` to apply various performance optimizations. All its configuration items are as follows:


| Node                          | Type      | Optional | Default | Purpose |
| ----                          | ----      | ----     | ----    | ---- |
| timeout_executor              | string    | Optional | ""      | Executor for RPC timeout on Client side |


Usage notes are as follows:
- The Server's executor will use the executor at the time of Client invocation. Similarly, the executor after Client invocation will use the executor at the time of Server return.
- If Client and Server are in one Pkg, the Req and Rsp will be passed directly via pointers; if Client and Server are in the same AimRT process but in different Pkgs, the Req and Rsp will undergo one serialization/deserialization before being passed.
- The timeout function only takes effect when Client and Server are in different Pkgs. If Client and Server are in one Pkg, for performance optimization, the Req and Rsp pointers will be passed directly. In this case, to ensure the Client-side Req and Rsp lifecycle covers the Server-side Req and Rsp lifecycle, the timeout function will not take effect.



Here is a simple example:

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
