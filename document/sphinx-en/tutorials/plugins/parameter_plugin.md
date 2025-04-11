

# Parameter Plugin

## Related Links

Protocol File:
- {{ '[parameter.proto]({}/src/protocols/plugins/parameter_plugin/parameter.proto)'.format(code_site_root_path_url) }}

Example Reference:
- {{ '[parameter_plugin]({}/src/examples/plugins/parameter_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

The **parameter_plugin** registers an RPC based on protobuf protocol definitions, providing management interfaces for Parameters. Note that **parameter_plugin** does not provide any communication backend itself, therefore this plugin should generally be used in conjunction with RPC backends from other communication plugins, such as the http RPC backend in [net_plugin](./net_plugin.md).

The plugin configuration options are as follows:

| Node                | Type          | Optional | Default | Description |
| ----                | ----          | ----     | ----    | ----        |
| service_name        | string        | Yes      | ""      | RPC Service Name, uses protocol-generated default value if empty |

Here's a simple configuration example combining **parameter_plugin** with the http RPC backend from **net_plugin**:

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
      - name: parameter_plugin
        path: ./libaimrt_parameter_plugin.so
  rpc:
    backends:
      - type: http
    servers_options:
      - func_name: "(pb:/aimrt.protocols.parameter_plugin.*)"
        enable_backends: [http]
```

## ParameterService

The {{ '[parameter.proto]({}/src/protocols/plugins/parameter_plugin/parameter.proto)'.format(code_site_root_path_url) }} protocol file defines a `ParameterService` with the following interfaces:
- **Set**: Set parameters
- **Get**: Retrieve parameters
- **List**: List parameter keys
- **Dump**: Export all parameters
- **Load**: Load parameter package (can load previously dumped parameters)

### Set

The `Set` interface is used to set/update a Key-Val parameter pair for a module. Interface definition:
```proto
message SetParameterReq {
  string module_name = 1;
  string parameter_key = 2;
  string parameter_value = 3;
}

message SetParameterRsp {
  uint64 code = 1;
  string msg = 2;
}

service ParameterService {
  // ...
  rpc Set(SetParameterReq) returns (SetParameterRsp);
  // ...
}
```

Example using curl with http RPC backend from **net_plugin**:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.parameter_plugin.ParameterService/Set' \
    -d '{"module_name": "ParameterModule", "parameter_key": "key-1", "parameter_value": "val-abc"}'
```

This command adds/updates a parameter pair (Key: `key-1`, Val: `val-abc`) in the `ParameterModule`. Successful response:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 21

{"code":0,"msg":""}
```

### Get

The `Get` interface retrieves a Key-Val parameter pair from a module. Interface definition:
```proto
message GetParameterReq {
  string module_name = 1;
  string parameter_key = 2;
}

message GetParameterRsp {
  uint32 code = 1;
  string msg = 2;
  string parameter_value = 3;
}

service ParameterService {
  // ...
  rpc Get(GetParameterReq) returns (GetParameterRsp);
  // ...
}
```

Example using curl with http RPC backend:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.parameter_plugin.ParameterService/Get' \
    -d '{"module_name": "ParameterModule", "parameter_key": "key-1"}'
```

This command retrieves the value of Key `key-1` from `ParameterModule`. Successful response:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 47

{"code":0,"msg":"","parameter_value":"val-abc"}
```

### List

The `List` interface retrieves all parameter keys from a module. Interface definition:
```proto
message ListParameterReq {
  string module_name = 1;
}

message ListParameterRsp {
  uint32 code = 1;
  string msg = 2;
  repeated string parameter_keys = 3;
}

service ParameterService {
  // ...
  rpc List(ListParameterReq) returns (ListParameterRsp);
  // ...
}
```

Example using curl with http RPC backend:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.parameter_plugin.ParameterService/List' \
    -d '{"module_name": "ParameterModule"}'
```

This command lists all keys in `ParameterModule`. Successful response:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 62

{"code":0,"msg":"","parameter_keys":["key-3","key-2","key-1"]}
```

### Dump

The `Dump` interface exports all Key-Val pairs from specified modules. Interface definition:
```proto
message ParameterMap {
  map<string, string> value = 1;
}

message DumpParameterReq {
  repeated string module_names = 1;
}

message DumpParameterRsp {
  uint32 code = 1;
  string msg = 2;

  map<string, ParameterMap> module_parameter_map = 3;  // key: module_name
}

service ParameterService {
  // ...
  rpc Dump(DumpParameterReq) returns (DumpParameterRsp);
  // ...
}
```

Example dumping parameters from `ParameterModule` using curl:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.parameter_plugin.ParameterService/Dump' \
    -d '{"module_names": ["ParameterModule"]}'
```

Successful response:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 218

{
	"code": 0,
	"msg": "",
	"module_parameter_map": {
		"ParameterModule": {
			"value": {
				"key-3": "val-3",
				"key-9": "val-9",
				"key-8": "val-8",
				"key-4": "val-4",
				"key-5": "val-5",
				"key-2": "val-2",
				"key-1": "val-1",
				"key-7": "val-7",
				"key-6": "val-6"
			}
		}
	}
}
```

### Load

The `Load` interface imports parameter packages (compatible with `Dump` output) to modules. Interface definition:
```proto
message ParameterMap {
  map<string, string> value = 1;
}

message LoadParameterReq {
  map<string, ParameterMap> module_parameter_map = 1;  // key: module_name
}

message LoadParameterRsp {
  uint32 code = 1;
  string msg = 2;
}

service ParameterService {
  // ...
  rpc Load(LoadParameterReq) returns (LoadParameterRsp);
  // ...
}
```

Example importing parameters to `ParameterModule` using curl:
```shell
#!/bin/bash

data='{
	"module_parameter_map": {
		"ParameterModule": {
			"value": {
				"key-a": "val-111",
				"key-b": "val-222",
				"key-c": "val-333",
				"key-d": "val-444",
				"key-e": "val-555",
				"key-f": "val-666",
				"key-g": "val-777"
			}
		}
	}
}'


curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.parameter_plugin.ParameterService/Load' \
    -d "$data"
```

Successful response:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```