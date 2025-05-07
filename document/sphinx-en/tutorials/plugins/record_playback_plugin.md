

# Record Playback Plugin

## Related Links

Protocol File:
- {{ '[record_playback.proto]({}/src/protocols/plugins/record_playback_plugin/record_playback.proto)'.format(code_site_root_path_url) }}

Reference Example:
- {{ '[record_playback_plugin]({}/src/examples/plugins/record_playback_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

**record_playback_plugin** is used for recording and playback of Channel data. The plugin supports loading independent type_support_pkg and multiple working modes including immediate mode and signal-triggered mode.

When in use, the plugin registers one or more Channel Subscribers/Publishers according to configurations to store subscribed data into databases or read data from databases for publishing. Additionally, the plugin registers an RPC based on protobuf protocol definition, providing interfaces for signal-triggered mode. Note that **record_playback_plugin** does not provide any communication backend, therefore signal triggering functionality generally needs to be used with RPC backends from other communication plugins, such as the http RPC backend in [net_plugin](./net_plugin.md).

The plugin configuration items are as follows:

| Node                              | Type          | Optional | Default  | Purpose |
| ----                              | ----          | ----  | ----      | ---- |
| service_name                      | string        | Yes   | ""        | RPC Service Name, uses protocol-generated default if empty |
| type_support_pkgs                 | array         | Yes   | []        | type support package configuration |
| type_support_pkgs[i].path         | string        | Required | ""        | Path to type support package |
| timer_executor                    | string        | Required in recording mode | []        | Executor for recording, must support time schedule |
| record_actions                    | array         | Yes   | []        | Recording action configuration |
| record_actions[i].name            | string        | Required | ""        | Action name |
| record_actions[i].options         | map           | Required | -         | Action options |
| record_actions[i].options.bag_path        | string        | Required | ""        | Path to store recording package |
| record_actions[i].options.mode            | string        | Required | ""        | Recording mode (case-insensitive): Immediate - "imd", Signal-triggered - "signal" |
| record_actions[i].options.max_preparation_duration_s  | unsigned int  | Yes   | 0      | Maximum data preparation time in advance (only effective in signal mode) |
| record_actions[i].options.executor        | string        | Required | ""        | Recording executor, must be thread-safe |
| record_actions[i].options.storage_policy  | map           | Yes   | -         | Storage policy for recording packages |
| record_actions[i].options.storage_policy.max_bag_size_m  | unsigned int  | Yes   | 2048      | Maximum recording package size in MB |
| record_actions[i].options.storage_policy.max_bag_num     | unsigned int  | Yes   | 0         | Maximum number of packages (oldest deleted when exceeded). 0 means unlimited |
| record_actions[i].options.storage_policy.msg_write_interval     | unsigned int  | Yes   | 1000         | Force disk write after receiving N messages |
| record_actions[i].options.storage_policy.msg_write_interval_time     | unsigned int  | Yes   | 1000         | Force disk write periodically (default unit: ms) |
| record_actions[i].options.storage_policy.compression_mode | string        | Yes   | zstd        | Compression mode (only valid for mcap format, case-insensitive): none, lz4, zstd |
| record_actions[i].options.storage_policy.compression_level | string        | Yes   |   default    | Compression level (only valid for mcap format, case-insensitive): fastest, fast, default, slow, slowest |
| record_actions[i].options.extra_attributes                | map     | Yes   | []          | Additional properties for recording |
| record_actions[i].options.topic_meta_list | array        | Yes   | []        | Topics and types to record |
| record_actions[i].options.topic_meta_list[j].topic_name   | string        | Required | ""        | Topic to record |
| record_actions[i].options.topic_meta_list[j].msg_type     | string        | Required | ""        | Message type to record |
| record_actions[i].options.topic_meta_list[j].serialization_type     | string        | Yes   | ""        | Serialization type for recording (uses default if empty) |
| playback_actions                  | array         | Yes   | []        | Playback action configuration |
| playback_actions[i].name          | string        | Required | ""        | Action name |
| playback_actions[i].options       | map           | Required | -         | Action options |
| playback_actions[i].options.bag_path      | string        | Required | ""        | Path to recording package |
| playback_actions[i].options.mode          | string        | Required | ""        | Playback mode (case-insensitive): Immediate - "imd", Signal-triggered - "signal" |
| playback_actions[i].options.executor      | string        | Required | ""        | Playback executor, must support time schedule |
| playback_actions[i].options.skip_duration_s   | unsigned int  | Yes   | 0      | Skip duration during playback (only effective in imd mode) |

## Configuration Parameters

### Recording Configuration

| Parameter                        | Type          | Required | Default | Description                                                                 |
|----------------------------------|---------------|----------|---------|-----------------------------------------------------------------------------|
| record_actions                   | array         | Yes      | []      | List of recording actions                                                  |
| record_actions[i].name           | string        | Yes      | ""      | Action name, must be unique                                                |
| record_actions[i].mode           | string        | Yes      | "imd"   | Recording mode: "imd" (immediate) / "trigger" (signal triggered recording) |
| record_actions[i].thread_num     | int           | Optional | 1       | Number of recording threads                                                |
| record_actions[i].package_path   | string        | Yes      | ""      | Storage path for recording packages                                        |
| record_actions[i].options        | object        | Yes      | {}      | Recording options                                                           |
| record_actions[i].options.compress | bool        | Optional | true    | Enable MCAP compression                                                    |
| record_actions[i].options.compress_mode | string | Optional | "Zstd" | Compression algorithm: Zstd/Lz4                                            |
| record_actions[i].options.compression_level | string | Optional | "Fast" | Compression level: Fast/Fastest/Balanced/Best                              |
| record_actions[i].options.topic_meta_list | array | Optional | []      | List of topics to record                                                   |
| record_actions[i].options.topic_meta_list[j].topic_name | string | Yes | ""      | Topic name to record                                                       |
| record_actions[i].options.topic_meta_list[j].msg_type | string | Yes | ""      | Message type to record                                                     |

### Playback Configuration

| Parameter                        | Type          | Required | Default | Description                                                                 |
|----------------------------------|---------------|----------|---------|-----------------------------------------------------------------------------|
| playback_actions                 | array         | Yes      | []      | List of playback actions                                                   |
| playback_actions[i].name         | string        | Yes      | ""      | Action name, must be unique                                                |
| playback_actions[i].mode         | string        | Yes      | "imd"   | Playback mode: "imd" (immediate playback)                                  |
| playback_actions[i].thread_num   | int           | Optional | 1       | Number of playback threads                                                 |
| playback_actions[i].package_path | string       | Yes      | ""      | Path of the playback package                                               |
| playback_actions[i].options      | object        | Yes      | {}      | Playback options                                                           |
| playback_actions[i].options.play_duration_s | unsigned int | Optional | 0      | Playback duration (only effective in imd mode). 0 means play entire package |
| playback_actions[i].options.topic_meta_list | array | Optional | []    | Topics to playback (must exist in recording). Empty means play all         |
| playback_actions[i].options.topic_meta_list[j].topic_name | string | Yes | "" | Topic to playback                                                          |
| playback_actions[i].options.topic_meta_list[j].msg_type | string | Yes | "" | Message type to playback                                                  |

**Note:** The **record_playback_plugin** manages recording/playback operations in `action` units. Each recording/playback `action` can have independent modes, threads, package paths, and can be triggered separately. Allocate appropriate resources for each action based on actual data size and frequency.

The `record_playback_plugin` supports MCAP format for disk storage. Compression modes and levels can be configured according to [MCAP default storage parameters](https://github.com/foxglove/mcap/blob/releases/cpp/v1.4.0/cpp/mcap/include/mcap/writer.hpp). To ensure compatibility with `plotjuggler 3.9.1` visualization, both `publish time` and `log time` in MCAP files will be set to `log time` values.

Below is a sample configuration for signal-triggered recording:
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
      - name: record_playback_plugin
        path: ./libaimrt_record_playback_plugin.so
        options:
          timer_executor: storage_executor  # require time schedule!
          type_support_pkgs:
            - path: ./libexample_pb_ts.so
          record_actions:
            - name: my_signal_record
              options:
                bag_path: ./bag
                mode: signal # imd/signal
                max_preparation_duration_s: 10 # Effective only in signal mode
                executor: record_thread # require thread safe!
                storage_policy:
                  max_bag_size_m: 2048
                  msg_write_interval: 1000        # message count period
                  msg_write_interval_time: 1000   # ms
                  compression_mode: zstd     # comression mode
                  compression_level: default   # comression level
                extra_attributes:
                  platform: arm64
                  os: ubuntu-22.04
                topic_meta_list:
                  - topic_name: test_topic
                    msg_type: pb:aimrt.protocols.example.ExampleEventMsg
                    serialization_type: pb # optional
  executor:
    executors:
      - name: record_thread
        type: simple_thread
      - name: storage_executor
        type: asio_thread
        options:
          thread_num: 2
  channel:
    # ...
  rpc:
    backends:
      - type: http
    servers_options:
      - func_name: "(pb:/aimrt.protocols.record_playback_plugin.*)"
        enable_backends: [http]
```

## RecordPlaybackService

In {{ '[record_playback.proto]({}/src/protocols/plugins/record_playback_plugin/record_playback.proto)'.format(code_site_root_path_url) }}, a `RecordPlaybackService` is defined that provides the following interfaces:
- **StartRecord**: Initiates recording
- **StopRecord**: Terminates recording
- **StartPlayback**: Starts playback
- **StopPlayback**: Stops playback

### StartRecord

The `StartRecord` interface is used to initiate a signal mode record action. Its interface definition is as follows:
```proto
message StartRecordReq {
  string action_name = 1;
  uint32 preparation_duration_s = 2;
  uint32 record_duration_s = 3;  // record forever if value is 0
}

message CommonRsp {
  uint32 code = 1;
  string msg = 2;
}

service RecordPlaybackService {
  // ...
  rpc StartRecord(StartRecordReq) returns (CommonRsp);
  // ...
}
```

Developers can configure the following parameters in the request packet `StartRecordReq`:
- `action_name`: Name of the record action to start
- `preparation_duration_s`: Time duration (in seconds) to backtrack recording from the request reception moment. Must not exceed the `max_preparation_duration_s` value configured for the action
- `record_duration_s`: Recording duration in seconds. If set to 0, recording will continue indefinitely until process termination

The following example demonstrates using the curl tool via the HTTP RPC backend in the **net_plugin** to call this interface:
```shell
data='{
    "action_name": "my_signal_record",
    "preparation_duration_s": 5,
    "record_duration_s": 10
}'

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.record_playback_plugin.RecordPlaybackService/StartRecord' \
    -d "$data"
```

This command initiates a record action named `my_signal_record` with 5-second backtrack recording and 10-second duration. Successful invocation returns:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```

### StopRecord

The `StopRecord` interface is used to terminate a running signal mode record action. Its interface definition is as follows:
```proto
message StopRecordReq {
  string action_name = 1;
}

message CommonRsp {
  uint32 code = 1;
  string msg = 2;
}

service RecordPlaybackService {
  // ...
  rpc StopRecord(StopRecordReq) returns (CommonRsp);
  // ...
}
```

Developers can configure the following parameter in the request packet `StartRecordReq`:
- `action_name`: Name of the record action to stop

The following example demonstrates using the curl tool via the HTTP RPC backend in the **net_plugin** to call this interface:
```shell
data='{
    "action_name": "my_signal_record"
}'

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.record_playback_plugin.RecordPlaybackService/StopRecord' \
    -d "$data"
```

This command stops the record action named `my_signal_record`. Successful invocation returns:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```

### StartPlayback

The `StartPlayback` interface is used to initiate a signal mode playback action. Its interface definition is as follows:
```proto
message StartPlaybackReq {
  string action_name = 1;
  uint32 skip_duration_s = 2;
  uint32 play_duration_s = 3;  // playback to end if value is 0
}

message CommonRsp {
  uint32 code = 1;
  string msg = 2;
}

service RecordPlaybackService {
  // ...
  rpc StartPlayback(StartPlaybackReq) returns (CommonRsp);
  // ...
}
```

Developers can configure the following parameters in the request packet `StartPlaybackReq`:
- `action_name`: Name of the playback action to start
- `skip_duration_s`: Time duration (in seconds) to skip
- `play_duration_s`: Playback duration in seconds

The following example demonstrates using the curl tool via the HTTP RPC backend in the **net_plugin** to call this interface:
```shell
data='{
    "action_name": "my_signal_playback",
    "skip_duration_s": 5,
    "play_duration_s": 10
}'

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.record_playback_plugin.RecordPlaybackService/StartPlayback' \
    -d "$data"
```

This command initiates a playback action named `my_signal_playback` with 5-second data skipping and 10-second playback duration. Successful invocation returns:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```

### StopPlayback

The `StopPlayback` interface is used to stop a running signal mode playback action. Its interface definition is as follows:
```proto
message StopPlaybackReq {
  string action_name = 1;
}

message CommonRsp {
  uint32 code = 1;
  string msg = 2;
}

service RecordPlaybackService {
  // ...
  rpc StopPlayback(StopPlaybackReq) returns (CommonRsp);
  // ...
}
```

Developers can configure the following parameters in the request package `StopPlaybackReq`:
- `action_name`: The name of the playback action to be stopped;

Here is an example of invoking this interface via HTTP using the curl tool through the http RPC backend in **net_plugin**:
```shell
data='{
    "action_name": "my_signal_playback"
}'

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.record_playback_plugin.RecordPlaybackService/StopPlayback' \
    -d "$data"
```

This example command can stop the playback action named `my_signal_playback`. If the call succeeds, the command returns:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```