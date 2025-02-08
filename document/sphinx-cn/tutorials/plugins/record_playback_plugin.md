
# 录播插件

## 相关链接

协议文件：
- {{ '[record_playback.proto]({}/src/protocols/plugins/record_playback_plugin/record_playback.proto)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[record_playback_plugin]({}/src/examples/plugins/record_playback_plugin)'.format(code_site_root_path_url) }}



## 插件概述


**record_playback_plugin**用于对 Channel 数据进行录制和播放。插件支持加载独立的 type_support_pkg，并支持立即模式、信号触发模式等多种工作模式。

在使用时，插件会根据配置注册一个或多个 Channel Subscriber 或 Publisher，订阅数据存储到数据库中，或从数据库中读取数据进行发布。此外，插件还注册了一个基于 protobuf 协议定义的RPC，提供了信号触发模式下的一些接口。请注意，**record_playback_plugin**没有提供任何通信后端，因此信号触发功能一般要搭配其他通信插件的 RPC 后端一块使用，例如[net_plugin](./net_plugin.md)中的 http RPC 后端。


插件的配置项如下：

| 节点                              | 类型          | 是否可选| 默认值  | 作用 |
| ----                              | ----          | ----  | ----      | ---- |
| service_name                      | string        | 可选  | ""        | RPC Service Name，不填则使用根据协议生成的默认值 |
| type_support_pkgs                 | array         | 可选  | []        | type support 包配置 |
| type_support_pkgs[i].path         | string        | 必选  | ""        | type support 包的路径 |
| timer_executor                    | string        | 录制模式下必选  | []        | 录制使用的执行器，要求必须支持 time schedule |
| record_actions                    | array         | 可选  | []        | 录制动作配置 |
| record_actions[i].name            | string        | 必选  | ""        | 动作名称 |
| record_actions[i].options         | map           | 必选  | -         | 动作选项 |
| record_actions[i].options.bag_path        | string        | 必选  | ""        | 录制包存放的路径 |
| record_actions[i].options.mode            | string        | 必选  | ""        | 录制模式，不区分大小写，立即模式：imd，信号触发模式：signal |
| record_actions[i].options.max_preparation_duration_s  | unsigned int  | 可选  | 0      | 最大提前数据预备时间，仅 signal 模式下生效 |
| record_actions[i].options.executor        | string        | 必选  | ""        | 录制使用的执行器，要求必须是线程安全的 |
| record_actions[i].options.storage_policy  | map           | 可选  | -         | 录制包的存储策略 |
| record_actions[i].options.storage_policy.storage_format  | string        | 可选  | mcap         | 录制包的存储格式, 目前支持 mcap 和 sqlite3 |
| record_actions[i].options.storage_policy.max_bag_size_m  | unsigned int  | 可选  | 2048      | 录制包 db 最大尺寸，单位 MB |
| record_actions[i].options.storage_policy.max_bag_num     | unsigned int  | 可选  | 0         | 录制包的最大个数，超出后将删除最早的包。0 表示无限大 |
| record_actions[i].options.storage_policy.msg_write_interval     | unsigned int  | 可选  | 1000         | 每收到多少消息提交一次事务 |
| record_actions[i].options.storage_policy.msg_write_interval_time     | unsigned int  | 可选  | 1000         | 每过多少时间提交一次事务，默认单位 ms|
| record_actions[i].options.storage_policy.journal_mode | string        | 可选  | memory        | sqlite3 日志模式，不区分大小写，现存在 [delete、truncate、persist、memory、wal、off](https://www.sqlite.org/pragma.html#pragma_journal_mode) 六种模式|
| record_actions[i].options.storage_policy.synchronous_mode | string        | 可选  |   full    | sqlite3 同步模式，不区分大小写，现存在 [extra、full、normal、off](https://www.sqlite.org/pragma.html#pragma_synchronous) 四种模式 |
| record_actions[i].options.topic_meta_list | array        | 可选  | []        | 要录制的 topic 和类型 |
| record_actions[i].options.topic_meta_list[j].topic_name   | string        | 必选  | ""        | 要录制的 topic |
| record_actions[i].options.topic_meta_list[j].msg_type     | string        | 必选  | ""        | 要录制的消息类型 |
| record_actions[i].options.topic_meta_list[j].serialization_type     | string        | 可选  | ""        | 录制时使用的序列化类型，不填则使用该消息类型的默认序列化类型 |
| playback_actions                  | array         | 可选  | []        | 播放动作配置 |
| playback_actions[i].name          | string        | 必选  | ""        | 动作名称 |
| playback_actions[i].options       | map           | 必选  | -         | 动作选项 |
| playback_actions[i].options.bag_path      | string        | 必选  | ""        | 录制包的路径 |
| playback_actions[i].options.mode          | string        | 必选  | ""        | 播放模式，不区分大小写，立即模式：imd，信号触发模式：signal |
| playback_actions[i].options.executor      | string        | 必选  | ""        | 播放使用的执行器，要求必须支持 time schedule |
| playback_actions[i].options.skip_duration_s   | unsigned int  | 可选  | 0      | 播放时跳过的时间，仅 imd 模式下生效 |
| playback_actions[i].options.play_duration_s   | unsigned int  | 可选  | 0      | 播放时长，仅 imd 模式下生效。0 表示播完整个包 |
| playback_actions[i].options.topic_meta_list   | array         | 可选  | []    | 要播放的 topic 和类型，必须要在录制包中存在。如果不填，则默认播放所有 |
| playback_actions[i].options.topic_meta_list[j].topic_name   | string        | 必选  | ""        | 要播放的 topic |
| playback_actions[i].options.topic_meta_list[j].msg_type     | string        | 必选  | ""        | 要播放的消息类型 |



请注意，**record_playback_plugin**中是以`action`为单元管理录制、播放动作的，每个录制/播放`action`可以有自己的模式、线程、包路径等参数，也可以独立触发。使用时可以根据数据实际大小和频率，为每个 action 分配合理的资源。

`record_playback_plugin` 的录制模式支持配置落盘格式，当前支持的格式有： `mcap` 和 `sqlite3` 格式，在不配置`storage_policy`的时候，默认落盘格式为 `mcap`：
- `sqlite3` 支持日志模式配置和同步模式配置，关于 `sqlite3` 模式的不同可以参考 [sqlite3 journal mode](https://www.sqlite.org/pragma.html#pragma_journal_mode)和 [sqlite3 synchronous](https://www.sqlite.org/pragma.html#pragma_synchronous)；
- `mcap`暂时不支持配置，均为默认参数，可参考 [mcap 默认落盘参数](https://github.com/foxglove/mcap/blob/releases/cpp/v1.4.0/cpp/mcap/include/mcap/writer.hpp)；为了适配 `plotjuggler 3.9.1`，`mcap`落盘数据时，`publish time`和`log time`均被设置为`log time`。

以下是一个信号触发录制功能的简单示例配置：
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
            - path: ./libexample_event_ts_pkg.so
          record_actions:
            - name: my_signal_record
              options:
                bag_path: ./bag
                mode: signal # imd/signal
                max_preparation_duration_s: 10 # Effective only in signal mode
                executor: record_thread # require thread safe!
                storage_policy:
                  storage_format: mcap
                  max_bag_size_m: 2048
                  msg_write_interval: 1000        # message count period
                  msg_write_interval_time: 1000   # ms
                  journal_mode: WAL
                  synchronous_mode: full
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

在{{ '[record_playback.proto]({}/src/protocols/plugins/record_playback_plugin/record_playback.proto)'.format(code_site_root_path_url) }}中，定义了一个`RecordPlaybackService`，提供了如下接口：
- **StartRecord**：开始录制；
- **StopRecord**：结束录制；
- **StartPlayback**：开始播放；
- **StopPlayback**：结束播放；

### StartRecord


`StartRecord`接口用于启动某个 signal 模式的 record action，其接口定义如下：
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

开发者可以在请求包`StartRecordReq`中填入以下参数：
- `action_name`：想要启动的 record action 名称；
- `preparation_duration_s`：收到请求时，向前回溯录制的时间，单位：s，最大不能超过 action 配置时的`max_preparation_duration_s`值；
- `record_duration_s`：录制持续的时间，单位：s，如果为 0，则会一直录制下去，直到进程停止；


以下是一个基于**net_plugin**中的 http RPC 后端，使用 curl 工具通过 Http 方式调用该接口的一个示例：
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

该示例命令可以启动名称为`my_signal_record`的 record action，向前回溯录制 5s 的数据，录制持续时间 10s。如果调用成功，该命令返回值如下：
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```


### StopRecord

`StopRecord`接口用于停止某个正在运行的 signal 模式的 record action，其接口定义如下：
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

开发者可以在请求包`StartRecordReq`中填入以下参数：
- `action_name`：想要停止的 record action 名称；


以下是一个基于**net_plugin**中的 http RPC 后端，使用 curl 工具通过 Http 方式调用该接口的一个示例：
```shell
data='{
    "action_name": "my_signal_record"
}'

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.record_playback_plugin.RecordPlaybackService/StopRecord' \
    -d "$data"
```

该示例命令可以停止名称为`my_signal_record`的 record action。如果调用成功，该命令返回值如下：
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```


### StartPlayback


`StartPlayback`接口用于启动某个 signal 模式的 playback action，其接口定义如下：
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

开发者可以在请求包`StartPlaybackReq`中填入以下参数：
- `action_name`：想要启动的 playback action 名称；
- `skip_duration_s`：跳过的时间，单位：s；
- `play_duration_s`：播放持续的时间，单位：s；


以下是一个基于**net_plugin**中的 http RPC 后端，使用 curl 工具通过 Http 方式调用该接口的一个示例：
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

该示例命令可以启动名称为`my_signal_playback`的 playback action，跳过 5s 的数据，播放持续时间 10s。如果调用成功，该命令返回值如下：
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```


### StopPlayback


`StopPlayback`接口用于停止某个正在运行的 signal 模式的 playback action，其接口定义如下：
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

开发者可以在请求包`StopPlaybackReq`中填入以下参数：
- `action_name`：想要停止的 playback action 名称；


以下是一个基于**net_plugin**中的 http RPC 后端，使用 curl 工具通过 Http 方式调用该接口的一个示例：
```shell
data='{
    "action_name": "my_signal_playback"
}'

curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.record_playback_plugin.RecordPlaybackService/StopPlayback' \
    -d "$data"
```

该示例命令可以停止名称为`my_signal_playback`的 playback action。如果调用成功，该命令返回值如下：
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 19

{"code":0,"msg":""}
```
