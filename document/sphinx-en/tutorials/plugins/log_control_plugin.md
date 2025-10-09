# Log Dynamic Control Plugin


## Related Links

Protocol file:
- {{ '[log_control.proto]({}/src/protocols/plugins/log_control_plugin/log_control.proto)'.format(code_site_root_path_url) }}

Reference example:
- {{ '[log_control_plugin]({}/src/examples/plugins/log_control_plugin)'.format(code_site_root_path_url) }}


## Plugin Overview


The **log_control_plugin** registers an RPC based on a protobuf-defined protocol, providing runtime management interfaces for Log. Note that the **log_control_plugin** does not provide any communication backend, so this plugin generally needs to be used together with the RPC backend of other communication plugins, such as the http RPC backend in [net_plugin](./net_plugin.md).


The plugin configuration items are as follows:

| Node                              | Type          | Optional | Default   | Purpose |
| ----                              | ----          | ----     | ----      | ---- |
| service_name                      | string        | Optional | ""        | RPC Service Name; if not filled, the default value generated according to the protocol will be used |


Below is a simple configuration example, combining **log_control_plugin** with the http RPC backend in **net_plugin**:



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
- **GetModuleLogLevel**: Get module log level;
- **SetModuleLogLevel**: Set module log level;


### GetModuleLogLevel

The `GetModuleLogLevel` interface is used to get the log level of a certain module, and its interface definition is as follows:

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


Developers fill in the module whose log level they want to query in the request packet `GetModuleLogLevelReq`. If it is empty, all modules are returned.


Below is an example of calling this interface via Http using the curl tool, based on the http RPC backend in **net_plugin**:

```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.log_control_plugin.LogControlService/GetModuleLogLevel' \
    -d '{"module_names": []}'
```


This example command queries the log levels of all current modules. If the call is successful, the command returns the following:

```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 84

{"code":0,"msg":"","module_log_level_map":{"core":"Info","ExecutorCoModule":"Info"}}
```



### SetModuleLogLevel


The `SetModuleLogLevel` interface is used to set the log level of one or more modules, and its interface definition is as follows:

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


Developers fill in the modules whose log levels they want to set and the corresponding log levels in the request packet `SetModuleLogLevelReq`.


Below is an example of calling this interface via Http using the curl tool, based on the http RPC backend in **net_plugin**:

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
