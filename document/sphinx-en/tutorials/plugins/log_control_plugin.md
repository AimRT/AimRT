# Log Dynamic Control Plugin

## Related Links

Protocol File:
- {{ '[log_control.proto]({}/src/protocols/plugins/log_control_plugin/log_control.proto)'.format(code_site_root_path_url) }}

Reference Example:
- {{ '[log_control_plugin]({}/src/examples/plugins/log_control_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

The **log_control_plugin** registers an RPC based on protobuf protocol definitions, providing runtime management interfaces for Log. Note that **log_control_plugin** does not provide any communication backend, so this plugin is typically used in combination with RPC backends from other communication plugins, such as the HTTP RPC backend in [net_plugin](./net_plugin.md).

The plugin configuration items are as follows:

| Node                              | Type          | Optional | Default  | Purpose |
| ----                              | ----          | ----     | ----     | ----    |
| service_name                      | string        | Yes      | ""       | RPC Service Name, if left blank, uses the default value generated from the protocol |

Here is a simple configuration example combining **log_control_plugin** with the HTTP RPC backend from **net_plugin**:

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

In {{ '[log_control.proto]({}/src/protocols/plugins/log_control_plugin/log_control.proto)'.format(code_site_root_path_url) }}, a `LogControlService` is defined, providing the following interfaces:
- **GetModuleLogLevel**: Get the log level of a module;
- **SetModuleLogLevel**: Set the log level of a module;

### GetModuleLogLevel

The `GetModuleLogLevel` interface is used to retrieve the log level of a specific module. Its interface definition is as follows:
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

Developers fill in the module they want to query for the log level in the request packet `GetModuleLogLevelReq`. If left empty, it returns all modules.

Here is an example using the HTTP RPC backend from **net_plugin**, calling this interface via HTTP using the curl tool:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.log_control_plugin.LogControlService/GetModuleLogLevel' \
    -d '{"module_names": []}'
```

This example command queries the current log levels of all modules. If the call is successful, the command returns the following:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 84

{"code":0,"msg":"","module_log_level_map":{"core":"Info","ExecutorCoModule":"Info"}}
```

### SetModuleLogLevel

The `SetModuleLogLevel` interface is used to set the log level of one or more modules. Its interface definition is as follows:
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

Developers fill in the module(s) they want to set the log level for and the corresponding log level in the request packet `SetModuleLogLevelReq`.

Here is an example using the HTTP RPC backend from **net_plugin**, calling this interface via HTTP using the curl tool:
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

This example command sets the log level to `Trace` for the `core` and `ExecutorCoModule` modules. If the call is successful, the command returns the following:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```