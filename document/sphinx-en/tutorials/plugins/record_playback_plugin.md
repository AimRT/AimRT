# Record and Playback Plugin

## Related Links

Protocol file:
- {{ '[record_playback.proto]({}/src/protocols/plugins/record_playback_plugin/record_playback.proto)'.format(code_site_root_path_url) }}

Reference example:
- {{ '[record_playback_plugin]({}/src/examples/plugins/record_playback_plugin)'.format(code_site_root_path_url) }}## Plugin Overview

**record_playback_plugin** is used for recording and playback of Channel data. The plugin supports loading independent type_support_pkg and supports multiple working modes such as immediate mode and signal trigger mode; it also supports frame extraction for a single topic.

Immediate mode: Start saving to disk immediately after the program starts;
Signal trigger mode: Only start saving to disk after receiving an RPC request for "start saving"; each request creates a new folder by default.

When in use, the plugin will register one or more Channel Subscribers or Publishers according to the configuration, subscribing to data to store in the database, or reading data from the database for publishing. Additionally, the plugin registers an RPC based on the protobuf protocol definition, providing some interfaces for signal trigger mode. Please note that **record_playback_plugin** does not provide any communication backend, so the signal trigger functionality generally needs to be used together with the RPC backend of other communication plugins, such as the http RPC backend in [net_plugin](./net_plugin.md).

The plugin configuration items are as follows:

| Node                              | Type          | Optional | Default   | Function |
| ----                              | ----          | ----     | ----      | ---- |
| service_name                      | string        | Optional | ""        | RPC Service Name, if not filled, the default value generated according to the protocol will be used |
| type_support_pkgs                 | array         | Optional | []        | type support package configuration |
| type_support_pkgs[i].path         | string        | Required | ""        | Path of the type support package |
| timer_executor                    | string        | Required in recording mode | []        | Executor used for recording, must support time schedule |
| record_actions                    | array         | Optional | []        | Recording action configuration |
| record_actions[i].name            | string        | Required | ""        | Action name |
| record_actions[i].options         | map           | Required | -         | Action options |
| record_actions[i].options.bag_path        | string        | Required | ""        | Path where the recording package is stored |
| record_actions[i].options.mode            | string        | Required | ""        | Recording mode, case-insensitive, immediate mode: imd, signal trigger mode: signal |
| record_actions[i].options.max_preparation_duration_s  | unsigned int  | Optional | 0      | Maximum pre-data preparation time, only effective in signal mode |
| record_actions[i].options.executor        | string        | Required | ""        | Executor used for recording, must be thread-safe |
| record_actions[i].options.storage_policy  | map           | Optional | -         | Storage policy for the recording package |
| record_actions[i].options.storage_policy.max_bag_size_m  | unsigned int  | Optional | 2048      | Maximum size of the recording package, in MB |
| record_actions[i].options.storage_policy.max_bag_num     | unsigned int  | Optional | 0         | Maximum number of recording packages, the oldest package will be deleted when exceeded. 0 means unlimited |
| record_actions[i].options.storage_policy.msg_write_interval     | unsigned int  | Optional | 1000         | Force disk write every how many messages received |
| record_actions[i].options.storage_policy.msg_write_interval_time     | unsigned int  | Optional | 1000         | Force disk write every how much time passes, default unit ms |
| record_actions[i].options.storage_policy.compression_mode | string        | Optional | zstd        | Compression mode, only effective in mcap mode, case-insensitive, currently available: none, lz4, zstd |
| record_actions[i].options.storage_policy.compression_level | string        | Optional |   default    | Compression level, only effective in mcap mode, case-insensitive, currently available: fastest, fast, default, slow, slowest |
| record_actions[i].options.extra_attributes                | map     | Optional | []          | List of attributes attached during recording |
| record_actions[i].options.topic_meta_list | array        | Optional | []        | Topics and types to be recorded |
| record_actions[i].options.topic_meta_list[j].topic_name   | string        | Required | ""        | Topic to be recorded |
| record_actions[i].options.topic_meta_list[j].msg_type     | string        | Required | ""        | Message type to be recorded |
| record_actions[i].options.topic_meta_list[j].serialization_type     | string        | Optional | ""        | Serialization type used during recording, if not filled, the default serialization type of the message type will be used |
| record_actions[i].options.topic_meta_list[j].record_enabled         | string        | Optional | "true"    | Whether to enable recording for the topic by default (true means enabled, false means disabled) |
| record_actions[i].options.topic_meta_list[j].sample_freq            | double        | Optional | ""        | Expected disk write frequency during recording; if actual frequency is higher than expected, frames will be extracted, if lower or not filled, no extraction will occur |
| playback_actions                  | array         | Optional | []        | Playback action configuration |
| playback_actions[i].name          | string        | Required | ""        | Action name |
| playback_actions[i].options       | map           | Required | -         | Action options |
| playback_actions[i].options.bag_path      | string        | Required | ""        | Path of the recording package |
| playback_actions[i].options.mode          | string        | Required | ""        | Playback mode, case-insensitive, immediate mode: imd, signal trigger mode: signal |
| playback_actions[i].options.executor      | string        | Required | ""        | Executor used for playback, must support time schedule |
| playback_actions[i].options.skip_duration_s   | unsigned int  | Optional | 0      | Time to skip during playback, only effective in imd mode || playback_actions[i].options.play_duration_s   | unsigned int  | Optional  | 0      | Playback duration; only effective in imd mode. 0 means play the entire bag |
| playback_actions[i].options.topic_meta_list   | array         | Optional  | []    | Topics and types to play; must exist in the recorded bag. If omitted, all topics are played |
| playback_actions[i].options.topic_meta_list[j].topic_name   | string        | Required  | ""        | Topic to play |
| playback_actions[i].options.topic_meta_list[j].msg_type     | string        | Required  | ""        | Message type |



Note that in **record_playback_plugin**, recording and playback actions are managed in units of `action`; each recording/playback `action` can have its own mode, thread, bag path, and other parameters, and can also be triggered independently. When using it, you can allocate reasonable resources for each action based on the actual data size and frequency.

`record_playback_plugin` supports persisting to the `mcap` format and allows configuring compression mode and compression level; refer to [mcap default persistence parameters](https://github.com/foxglove/mcap/blob/releases/cpp/v1.4.0/cpp/mcap/include/mcap/writer.hpp). To be compatible with `plotjuggler 3.9.1` visualization, when persisting data to `mcap`, both `publish time` and `log time` are set to the value of `log time`.

Below is a simple example configuration for signal-triggered recording:

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
                    sample_freq: 10
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

In {{ '[record_playback.proto]({}/src/protocols/plugins/record_playback_plugin/record_playback.proto)'.format(code_site_root_path_url) }}, a `RecordPlaybackService` is defined, providing the following interfaces:
- **StartRecord**: Start recording;
- **StopRecord**: Stop recording;
- **StartPlayback**: Start playback;
- **StopPlayback**: Stop playback;
- **UpdateMetadata**: Update the `ext_attributes` field of the recording package;

### StartRecord


The `StartRecord` interface is used to start a signal-mode record action. Its interface definition is as follows:

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

message StartRecordRsp {
  CommonRsp common_rsp = 1;
  string filefolder = 2;
}

service RecordPlaybackService {
  // ...
  rpc StartRecord(StartRecordReq) returns (StartRecordRsp);
  // ...
}
```


Developers can fill in the following parameters in the request package `StartRecordReq`:
- `action_name`: The name of the record action to be started;
- `preparation_duration_s`: The time to look back and record when the request is received, in seconds. The maximum cannot exceed the `max_preparation_duration_s` value configured in the action;
- `record_duration_s`: The duration of the recording, in seconds. If it is 0, it will keep recording until the process stops;


Below is an example of calling this interface via HTTP using the curl tool, based on the http RPC backend in **net_plugin**:

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


This example command can start a record action named `my_signal_record`, looking back and recording 5s of data, with a recording duration of 10s. If the call is successful, the command returns the following, where `filefolder` indicates the folder path where the data is saved:

```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{
  "common_rsp": {
    "code": 0,
    "msg": ""
  },
  "filefolder": "/path/to/record/aimrtbag_YYYYMMDD_HHMMSS"
}
```



### StopRecord

The `StopRecord` interface is used to stop a running signal-mode record action. Its interface definition is as follows:

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


Developers can fill in the following parameters in the request package `StartRecordReq`:
- `action_name`: The name of the record action to be stopped;


Below is an example of calling this interface via HTTP using the curl tool, based on the http RPC backend in **net_plugin**:

```shell
data='{
    "action_name": "my_signal_record"
}'

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.record_playback_plugin.RecordPlaybackService/StopRecord' \
    -d "$data"
```


This example command can stop the record action named `my_signal_record`. If the call is successful, the command returns the following:

```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```



### StartPlayback


The `StartPlayback` interface is used to start a signal-mode playback action. Its interface definition is as follows:

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


Developers can fill in the following parameters in the request package `StartPlaybackReq`:
- `action_name`: The name of the playback action to be started;
- `skip_duration_s`: The time to skip, in seconds;
- `play_duration_s`: The duration of the playback, in seconds;


Below is an example of calling this interface via HTTP using the curl tool, based on the http RPC backend in **net_plugin**:

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


This example command can start a playback action named `my_signal_playback`, skipping 5s of data, with a playback duration of 10s. If the call is successful, the command returns the following:

```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```
### StopPlayback

The `StopPlayback` interface is used to stop a running playback action in signal mode. Its interface definition is as follows:

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


Developers can fill in the following parameters in the request package `StopPlaybackReq`:
- `action_name`: the name of the playback action to be stopped;

Below is an example of calling this interface via HTTP using the curl tool, based on the http RPC backend in **net_plugin**:

```shell
data='{
    "action_name": "my_signal_playback"
}'

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.record_playback_plugin.RecordPlaybackService/StopPlayback' \
    -d "$data"
```


This example command stops the playback action named `my_signal_playback`. If the call succeeds, the command returns the following:

```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```


### UpdateMetadata

The `UpdateMetadata` interface is used to update custom metadata (extra attributes) associated with a specified record action. This interface is not limited to any recording mode. Its interface definition is as follows:


```proto
message UpdateMetadataReq {
  string action_name = 1;
  map<string, string> kv_pairs = 2;
}

service RecordPlaybackService {
  // ...
  rpc UpdateMetadata(UpdateMetadataReq) returns (CommonRsp);
  // ...
}
```


Developers can fill in the following parameters in the request package `UpdateMetadataReq`:
- `action_name`: the name of the record action to be updated;
- `kv_pairs`: a map containing the metadata key-value pairs to be updated or added.
  - `key`: the name of the metadata. Note: when the key is an empty string, the key-value pair will be ignored; when keys are duplicated, only the last updated value is kept.
  - `value`: the value of the metadata. The server-side implementation of this interface will attempt to parse each value string as YAML:
    * If parsing succeeds, the value string is treated as valid YAML data. The parsed YAML structure (which may be a Map, List, or simple scalar such as string, number, boolean, etc.) will be stored as the metadata item. This means you can pass a serialized complex YAML structure string, and it will be stored as the corresponding structured data.
    * If parsing fails, the system will catch the parsing exception and store the original value string itself as a plain text string as the metadata item.

Below is an example of calling this interface via HTTP using the curl tool, based on the http RPC backend in **net_plugin**:

```shell
data='{
    "action_name": "my_signal_record",
    "kv_pairs":{ "key": "timestamp: '2023-10-25T12:34:56.789Z'\nposition:\n  x: 1.2\n  y: 3.4\n  z: 0.0\norientation:\n  roll: 0.0\n  pitch: 0.0\n  yaw: 1.57\nsensor_temperature: 25.5C\nsensor_distance_front: 1.8m\nbattery_voltage: 12.4V\nbattery_level: 85%\nmotor_speed_left: 100rpm\nmotor_speed_right: 102rpm\nstatus: active\nmode: autonomous\nlog_message: Navigation started."}
}'
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.record_playback_plugin.RecordPlaybackService/UpdateMetadata' \
    -d "$data"

```

This example command updates the metadata of the record action named `my_signal_record`. If the call succeeds, the command returns the following:


```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```



### UpdateRecordAction

The `UpdateRecordAction` interface is used to dynamically update the recording status of topics within a specified record action. This interface does not restrict the recording mode. The interface definition is as follows:

```proto
message TopicMeta {
  string topic_name = 1;
  string msg_type = 2;
  bool record_enabled = 3;
}

message UpdateRecordActionReq {
  string action_name = 1;
  repeated TopicMeta topic_metas = 2;
}

message UpdateRecordActionRsp {
  CommonRsp common_rsp = 1;
}

service RecordPlaybackService {
  // ...
  rpc UpdateRecordAction(UpdateRecordActionReq) returns (UpdateRecordActionRsp);
  // ...
}
```

Developers can fill in the following parameters in the request packet `UpdateRecordActionReq`:
- `action_name`: The name of the record action you want to update;
- `topic_metas`: A list containing the recording status to be updated for each topic. Each `TopicMeta` includes:
  - `topic_name`: The name of the topic to be updated;
  - `msg_type`: The message type corresponding to the topic;
  - `record_enabled`: Whether to enable recording for this topic (`true` means enabled, `false` means disabled).

**Notes:**
- Only topics that already exist in the record action configuration can be updated. If a non-existent topic is passed in, it will be ignored and a warning log will be printed.
- The update takes effect immediately and will impact subsequent data recording behavior.
- This interface is executed synchronously and will return only after the update is complete.

Below is an example of calling this interface via HTTP using the curl tool, based on the http RPC backend in **net_plugin**:
```shell
data='{
    "action_name": "my_imd_record",
    "topic_metas": [
        {
            "topic_name": "test_topic",
            "msg_type": "pb:aimrt.protocols.example.ExampleEventMsg",
            "record_enabled": false
        },
    ]
}'

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.record_playback_plugin.RecordPlaybackService/UpdateRecordAction' \
    -d "$data"
```

This example command updates the record action named `my_imd_record`, disabling the recording of `test_topic`. If the call succeeds, the command returns the following:

```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"common_rsp":{"code":0,"msg":""}}
```