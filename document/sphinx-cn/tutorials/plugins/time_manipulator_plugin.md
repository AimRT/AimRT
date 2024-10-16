# 时间管理器插件


## 相关链接

协议文件：
- {{ '[time_manipulator.proto]({}/src/protocols/plugins/time_manipulator_plugin/time_manipulator.proto)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[time_manipulator_plugin]({}/src/examples/plugins/time_manipulator_plugin)'.format(code_site_root_path_url) }}


## 插件概述


**time_manipulator_plugin**中提供了`time_manipulator`执行器，可以实现时间调速功能。同时注册了一个基于 protobuf 协议定义的 RPC，提供了对于`time_manipulator`执行器的一些管理接口。请注意，**time_manipulator_plugin**没有提供任何通信后端，因此本插件一般要搭配其他通信插件的 RPC 后端一块使用，例如[net_plugin](./net_plugin.md)中的 http RPC 后端。

插件的配置项如下：

| 节点                              | 类型          | 是否可选| 默认值  | 作用 |
| ----                              | ----          | ----  | ----      | ---- |
| service_name                      | string        | 可选  | ""        | RPC Service Name，不填则使用根据协议生成的默认值 |


以下是一个简单的配置示例，将**time_manipulator_plugin**与**net_plugin**中的 http RPC 后端搭配使用：

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
      - name: time_manipulator_plugin
        path: ./libaimrt_time_manipulator_plugin.so
  rpc:
    backends:
      - type: http
    servers_options:
      - func_name: "(pb:/aimrt.protocols.time_manipulator_plugin.*)"
        enable_backends: [http]
```


## time_manipulator 执行器


`time_manipulator`执行器是一种基于时间轮实现的可调速执行器，一般用于仿真、模拟等有定时任务、且对定时精度要求不高的场景。它会启动一个单独的线程跑时间轮，同时也支持将具体的任务投递到其他执行器中执行。其所有的配置项如下：

| 节点                  | 类型                  | 是否可选| 默认值 | 作用 |
| ----                  | ----                  | ----  | ----  | ---- |
| bind_executor         | string                | 必选  | ""    | 绑定的执行器 |
| dt_us                 | unsigned int          | 可选  | 1000  | 时间轮 tick 的间隔，单位：微秒 |
| init_ratio            | double                | 可选  | 1.0  | 初始时间系数 |
| wheel_size            | unsigned int array    | 可选  | [1000, 600]    | 各个时间轮的大小 |
| thread_sched_policy   | string                | 可选  | ""    | 时间轮线程的调度策略 |
| thread_bind_cpu       | unsigned int array    | 可选  | []    | 时间轮线程的绑核配置 |


使用注意点如下：
- `bind_executor`用于配置绑定的执行器，从而在时间达到后将任务投递到绑定的执行器里具体执行。
  - 必须要绑定其他执行器；
  - 线程安全性与绑定的执行器一致；
  - 如果绑定的执行器不存在，则会在初始化时抛出一个异常；
- `dt_us`是时间轮算法的一个参数，表示 Tick 的间隔。间隔越大，定时调度的精度越低，但越节省 CPU 资源。
- `init_ratio`是初始时间系数，默认为 1.0，表示与现实时间流速一致。
- `wheel_size`是时间轮算法的另一个参数，表示各个时间轮的大小。比如默认的参数`[1000, 600]`表示有两个时间轮，第一个轮的刻度是 1000，第二个轮的刻度是 600。如果 Tick 时间是 1ms，则第一个轮的完整时间是 1s，第二个轮的完整时间是 10min。一般来说，要让可能的定时时间都落在轮内最好。
- `thread_sched_policy`和`thread_bind_cpu`参考[Common Information](../cfg/common.md)中线程绑核配置的说明。


关于`time_manipulator`执行器的时间系数`time_ratio`是一个浮点型参数，说明如下：
- 如果`time_ratio`大于 1.0，则表示快进，流速是现实时间的`time_ratio`倍；
- 如果`time_ratio`等于 1.0，则表示与现实时间流速相等；
- 如果`time_ratio`大于 0.0 且 小于 1.0，则表示慢放，流速是现实时间的`time_ratio`倍；
- 如果`time_ratio`等于 0.0，则表示暂停；
- 如果`time_ratio`为负数，等效于 0.0，仍然表示暂停（时间无法倒退）；


`time_manipulator`执行器将在进程启动、执行器初始化时与现实时间同步一次，然后将按时间系数在执行器内部进行独立的时间流逝。参考[Executor](../interface_cpp/executor.md)接口文档，时间系数主要影响的是执行器的`Now`方法返回的时间，以及`ExecuteAt`、`ExecuteAfter`这两个方法执行任务时的时间。


以下是一个简单的示例：
```yaml
aimrt:
  plugin:
    plugins:
      - name: time_manipulator_plugin
        path: ./libaimrt_time_manipulator_plugin.so
  executor:
    executors:
      - name: real_work_thread_pool
        type: simple_thread
      - name: time_schedule_executor
        type: time_manipulator
        options:
          bind_executor: real_work_thread_pool
          dt_us: 1000
          init_ratio: 2.0
          wheel_size: [1000, 3600]
```


## TimeManipulatorService


在{{ '[time_manipulator.proto]({}/src/protocols/plugins/time_manipulator_plugin/time_manipulator.proto)'.format(code_site_root_path_url) }}协议文件中，定义了一个`TimeManipulatorService`，提供了如下接口：
- **SetTimeRatio**：设置时间系数；
- **Pause**：暂停；
- **GetTimeRatio**：获取当前时间系数；



### SetTimeRatio

`SetTimeRatio`接口用于为某个`time_manipulator`执行器设置时间系数，其接口定义如下：
```proto
message SetTimeRatioReq {
  string executor_name = 1;
  double time_ratio = 2;
}

message CommonRsp {
  double time_ratio = 1;
  uint32 code = 2;
  string msg = 3;
}

service TimeManipulatorService {
  // ...
  rpc SetTimeRatio(SetTimeRatioReq) returns (CommonRsp);
  // ...
}
```


以下是一个基于**net_plugin**中的 http RPC 后端，使用 curl 工具通过 Http 方式调用该接口的一个示例：
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.time_manipulator_plugin.TimeManipulatorService/SetTimeRatio' \
    -d '{"executor_name": "time_schedule_executor", "time_ratio": 2.0}'
```

该示例命令为`time_schedule_executor`这个执行器设置了时间系数为 2.0。如果设置成功，名称为`time_schedule_executor`的`time_manipulator`执行器时间系数将立即更新为 2，同时该命令返回值如下：
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 34

{"time_ratio":2,"code":0,"msg":""}
```

### Pause

`Pause`接口用于暂停某个`time_manipulator`执行器，其接口定义如下：
```proto
message PauseReq {
  string executor_name = 1;
}

message CommonRsp {
  double time_ratio = 1;
  uint32 code = 2;
  string msg = 3;
}

service TimeManipulatorService {
  // ...
  rpc Pause(PauseReq) returns (CommonRsp);
  // ...
}
```


以下是一个基于**net_plugin**中的 http RPC 后端，使用 curl 工具通过 Http 方式调用该接口的一个示例：
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.time_manipulator_plugin.TimeManipulatorService/Pause' \
    -d '{"executor_name": "time_schedule_executor"}'
```

该示例命令让`time_schedule_executor`这个执行器暂停时间流逝。如果设置成功，名称为`time_schedule_executor`的`time_manipulator`执行器将暂停，同时该命令返回值如下：
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 34

{"time_ratio":0,"code":0,"msg":""}
```

### GetTimeRatio

`GetTimeRatio`接口用于获取某个`time_manipulator`执行器当前的时间系数，其接口定义如下：
```proto
message GetTimeRatioReq {
  string executor_name = 1;
}
message CommonRsp {
  double time_ratio = 1;
  uint32 code = 2;
  string msg = 3;
}

service TimeManipulatorService {
  // ...
  rpc GetTimeRatio(GetTimeRatioReq) returns (CommonRsp);
  // ...
}
```


以下是一个基于**net_plugin**中的 http RPC 后端，使用 curl 工具通过 Http 方式调用该接口的一个示例：
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.time_manipulator_plugin.TimeManipulatorService/GetTimeRatio' \
    -d '{"executor_name": "time_schedule_executor"}'
```

该示例命令可以获取`time_schedule_executor`这个执行器当前的时间系数。如果获取成功，该命令返回值如下：
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 34

{"time_ratio":1,"code":0,"msg":""}
```

