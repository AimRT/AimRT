# Time Manipulator Plugin


## Related Links

Protocol file:
- {{ '[time_manipulator.proto]({}/src/protocols/plugins/time_manipulator_plugin/time_manipulator.proto)'.format(code_site_root_path_url) }}

Reference example:
- {{ '[time_manipulator_plugin]({}/src/examples/plugins/time_manipulator_plugin)'.format(code_site_root_path_url) }}


## Plugin Overview


The **time_manipulator_plugin** provides the `time_manipulator` executor, which can adjust the speed of time. It also registers an RPC based on a protobuf-defined protocol, offering management interfaces for the `time_manipulator` executor. Note that the **time_manipulator_plugin** does not provide any communication backend, so this plugin is generally used together with the RPC backend of another communication plugin, such as the HTTP RPC backend in [net_plugin](./net_plugin.md).

Plugin configuration items are as follows:

| Node                              | Type          | Optional | Default | Purpose |
| ----                              | ----          | ----     | ----    | ---- |
| service_name                      | string        | Optional | ""      | RPC Service Name; if left blank, the default value generated from the protocol is used |


Below is a simple configuration example that combines **time_manipulator_plugin** with the HTTP RPC backend in **net_plugin**:


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



## time_manipulator Executor


The `time_manipulator` executor is a timing wheel–based, speed-adjustable executor, typically used in simulation and emulation scenarios that have scheduled tasks and do not require high timing precision. It starts a dedicated thread to run the timing wheel and also supports dispatching specific tasks to other executors for execution. All its configuration items are as follows:

| Node                  | Type                  | Optional | Default | Purpose |
| ----                  | ----                  | ----     | ----    | ---- |
| bind_executor         | string                | Required | ""      | Bound executor |
| dt_us                 | unsigned int          | Optional | 100000  | Interval between timing-wheel ticks, in microseconds |
| init_ratio            | double                | Optional | 1.0     | Initial time ratio |
| wheel_size            | unsigned int array    | Optional | [100, 360] | Size of each timing wheel |
| thread_sched_policy   | string                | Optional | ""      | Scheduling policy for the timing-wheel thread |
| thread_bind_cpu       | unsigned int array    | Optional | []      | CPU affinity configuration for the timing-wheel thread |
| use_system_clock      | bool                  | Optional | false   | Whether to use std::system_clock; defaults to std::steady_clock |

Usage notes:
- `bind_executor` configures the executor to which tasks are dispatched once their scheduled time arrives.
  - You must bind another executor;
  - Thread safety is consistent with the bound executor;
  - If the bound executor does not exist, an exception is thrown during initialization;
- `dt_us` is a parameter of the timing-wheel algorithm, indicating the tick interval. A larger interval lowers scheduling precision but saves CPU resources.
- `init_ratio` is the initial time ratio, defaulting to 1.0, meaning the same flow rate as real time.
- `wheel_size` is another parameter of the timing-wheel algorithm, indicating the size of each wheel. For example, the default `[100, 360]` means two wheels: the first has 100 slots, the second has 360. If the tick interval is 1 ms, the first wheel covers 1 s and the second covers 10 min. Generally, it is best to ensure all possible scheduled times fall within the wheels.
- Refer to [Common Information](../cfg/common.md) for descriptions of `thread_sched_policy` and `thread_bind_cpu`.
- `use_system_clock` configures whether to use std::system_clock as the time source. The default is false, using std::steady_clock. Note that when std::system_clock is used, the executor’s time will synchronize with the system and may be affected by external adjustments.

Regarding the `time_ratio` parameter of the `time_manipulator` executor, a floating-point value:
- If `time_ratio` > 1.0, fast-forward; the flow rate is `time_ratio` times real time;
- If `time_ratio` == 1.0, the flow rate equals real time;
- If 0.0 < `time_ratio` < 1.0, slow-motion; the flow rate is `time_ratio` times real time;
- If `time_ratio` == 0.0, pause;
- If `time_ratio` is negative, it is treated as 0.0, still pausing (time cannot go backward);


The `time_manipulator` executor synchronizes with real time once at process startup (during executor initialization), then flows independently inside the executor according to the time ratio. Refer to the [Executor](../interface_cpp/executor.md) interface documentation: the time ratio mainly affects the value returned by the executor’s `Now` method and the scheduling time used by the `ExecuteAt` and `ExecuteAfter` methods.


Below is a simple example:

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


In the protocol file {{ '[time_manipulator.proto]({}/src/protocols/plugins/time_manipulator_plugin/time_manipulator.proto)'.format(code_site_root_path_url) }}, a `TimeManipulatorService` is defined, providing the following interfaces:
- **SetTimeRatio**: Set the time ratio;
- **Pause**: Pause;
- **GetTimeRatio**: Get the current time ratio;



### SetTimeRatio

The `SetTimeRatio` interface is used to set the time ratio for a `time_manipulator` executor. Its interface definition is as follows:

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



Below is an example using curl via HTTP, based on the HTTP RPC backend in **net_plugin**, to call this interface:

```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.time_manipulator_plugin.TimeManipulatorService/SetTimeRatio' \
    -d '{"executor_name": "time_schedule_executor", "time_ratio": 2.0}'
```


This example command sets the time ratio of the executor named `time_schedule_executor` to 2.0. If successful, the `time_manipulator` executor named `time_schedule_executor` will immediately update its time ratio to 2, and the command will return:

```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 34

{"time_ratio":2,"code":0,"msg":""}
```


### Pause

The `Pause` interface is used to pause a `time_manipulator` executor. Its interface definition is as follows:

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



Below is an example using curl via HTTP, based on the HTTP RPC backend in **net_plugin**, to call this interface:

```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.time_manipulator_plugin.TimeManipulatorService/Pause' \
    -d '{"executor_name": "time_schedule_executor"}'
```


This example command pauses the flow of time for the executor named `time_schedule_executor`. If successful, the `time_manipulator` executor named `time_schedule_executor` will pause, and the command will return:

```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 34

{"time_ratio":0,"code":0,"msg":""}
```


### GetTimeRatio

The `GetTimeRatio` interface is used to obtain the current time ratio of a `time_manipulator` executor. Its interface definition is as follows:

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



Below is an example using curl via HTTP, based on the HTTP RPC backend in **net_plugin**, to call this interface:

```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.time_manipulator_plugin.TimeManipulatorService/GetTimeRatio' \
    -d '{"executor_name": "time_schedule_executor"}'
```


This example command retrieves the current time ratio of the executor named `time_schedule_executor`. If successful, the command will return:

```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 34

{"time_ratio":1,"code":0,"msg":""}
```
