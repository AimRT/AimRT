

# Dynamic Log Control Plugin

## Relevant Links

Protocol Files:
- {{ '[log_control.proto]({}/src/protocols/plugins/log_control_plugin/log_control.proto)'.format(code_site_root_path_url) }}

Reference Example:
- {{ '[log_control_plugin]({}/src/examples/plugins/log_control_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

The **log_control_plugin** registers an RPC service based on protobuf protocol definitions, providing runtime management interfaces for logs. Note that **log_control_plugin** does not include any communication backend itself, therefore it typically needs to be used in conjunction with RPC backends from other communication plugins, such as the HTTP RPC backend in [net_plugin](./net_plugin.md).

Plugin configuration options:

| Node               | Type          | Optional | Default | Description |
|--------------------|---------------|----------|---------|-------------|
| service_name       | string        | Yes      | ""      | RPC Service Name. Uses protocol-generated default if empty |

Below is a configuration example combining **log_control_plugin** with the HTTP RPC backend from **net_plugin**:

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
      - name: log_control_plugin
        path: ./libaimrt_log_control_plugin.so
  rpc:
    backends:
      - type: http
    servers_options:
      - func_name: "(.*)"
        enable_backends: [http]
```

## LogControlService

The {{ '[log_control.proto]({}/src/protocols/plugins/log_control_plugin/log_control.proto)'.format(code_site_root_path_url) }} file defines a `LogControlService` with the following interfaces:
- **GetModuleLogLevel**: Retrieve the log level of a module
- **SetModuleLogLevel**: Set the log level for a module

### GetModuleLogLevel

The `GetModuleLogLevel` interface retrieves the log level of specified modules. Its definition:

```proto
message GetModuleLogLevelReq {
  repeated string module_names = 1;  // if empty, then get all module
}

message GetModuleLogLevelRsp {
  uint32 code = 1;
  string msg = 2;

  map<string, string> module_log_level_map = 3;  // key: module_name
}

service ParameterService {
  // ...
  rpc GetModuleLogLevel(GetModuleLogLevelReq) returns (GetModuleLogLevelRsp);
  // ...
}
```

Developers specify target modules in the `GetModuleLogLevelReq` request. Returns all modules if empty.

Example using curl with **net_plugin**'s HTTP RPC backend:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.log_control_plugin.LogControlService/GetModuleLogLevel' \
    -d '{"module_names": []}'
```

Successful execution returns:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 84

{"code":0,"msg":"","module_log_level_map":{"core":"Info","ExecutorCoModule":"Info"}}
```

### SetModuleLogLevel

The `SetModuleLogLevel` interface configures log levels for modules. Its definition:

```proto
message SetModuleLogLevelReq {
  map<string, string> module_log_level_map = 1;
}

message SetModuleLogLevelRsp {
  uint32 code = 1;
  string msg = 2;
}


service ParameterService {
  // ...
  rpc SetModuleLogLevel(SetModuleLogLevelReq) returns (SetModuleLogLevelRsp);
  // ...
}
```

Developers specify modules and corresponding log levels in `SetModuleLogLevelReq`.

Example using curl with **net_plugin**'s HTTP RPC backend:
```shell
#!/bin/bash

data='{
	"module_log_level_map": {
		"core": "Trace",
		"ExecutorCoModule": "Trace"
	}
}'

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.log_control_plugin.LogControlService/SetModuleLogLevel' \
    -d "$data"
```

This example sets `Trace` level for `core` and `ExecutorCoModule` modules. Successful execution returns:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```