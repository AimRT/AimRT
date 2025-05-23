# Time Manipulator Plugin

## Related Links

Protocol File:
- {{ '[time_manipulator.proto]({}/src/protocols/plugins/time_manipulator_plugin/time_manipulator.proto)'.format(code_site_root_path_url) }}

Reference Example:
- {{ '[time_manipulator_plugin]({}/src/examples/plugins/time_manipulator_plugin)'.format(code_site_root_path_url) }}


## Plugin Overview

The **time_manipulator_plugin** provides a `time_manipulator` executor that enables time scaling functionality. It also registers an RPC based on protobuf protocol definition, offering some management interfaces for the `time_manipulator` executor. Note that **time_manipulator_plugin** does not provide any communication backend, so this plugin is typically used in combination with RPC backends from other communication plugins, such as the http RPC backend in [net_plugin](./net_plugin.md).

The plugin configuration items are as follows:

| Node                              | Type          | Optional | Default  | Description |
| ----                              | ----          | ----     | ----     | ----        |
| service_name                      | string        | Yes      | ""       | RPC Service Name. If empty, uses the default value generated from the protocol |

Here's a simple configuration example combining **time_manipulator_plugin** with the http RPC backend from **net_plugin**:

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

The `time_manipulator` executor is a speed-adjustable executor based on a timing wheel implementation, typically used in simulation scenarios with timing tasks that don't require high timing precision. It runs a timing wheel in a separate thread while also supporting task delegation to other executors. All its configuration items are as follows:

| Node                  | Type                  | Optional | Default | Description |
| ----                  | ----                  | ----     | ----    | ----        |
| bind_executor         | string                | Required | ""      | Bound executor |
| dt_us                 | unsigned int          | Yes      | 100000  | Timing wheel tick interval in microseconds |
| init_ratio            | double                | Yes      | 1.0     | Initial time ratio |
| wheel_size            | unsigned int array    | Yes      | [100, 360] | Sizes of timing wheels |
| thread_sched_policy   | string                | Yes      | ""      | Scheduling policy for timing wheel thread |
| thread_bind_cpu       | unsigned int array    | Yes      | []      | CPU binding configuration for timing wheel thread |
| use_system_clock      | bool                  | Yes      | false   | Whether to use std::system_clock (default uses std::steady_clock) |

Usage notes:
- `bind_executor` configures the bound executor for task execution when time is reached.
  - Must bind to another executor;
  - Thread safety matches the bound executor;
  - Throws an exception during initialization if the bound executor doesn't exist;
- `dt_us` is a timing wheel algorithm parameter representing tick interval. Larger intervals mean lower scheduling precision but better CPU resource efficiency.
- `init_ratio` is the initial time ratio (default 1.0), meaning real-time speed.
- `wheel_size` is another timing wheel parameter representing wheel sizes. Default `[1000, 600]` means two wheels with 1000 and 600 ticks respectively. With 1ms tick time, the first wheel covers 1s and the second 10min. Generally, ensure possible timing ranges fall within the wheels.
- `thread_sched_policy` and `thread_bind_cpu` refer to thread binding configuration in [Common Information](../cfg/common.md).
- `use_system_clock` determines whether to use std::system_clock (default false uses std::steady_clock). Note that with std::system_clock, executor time synchronizes with system time and may be affected by external adjustments.

The `time_manipulator` executor's `time_ratio` parameter is a floating-point value with these meanings:
- `time_ratio` > 1.0: Fast-forward at `time_ratio` times real speed;
- `time_ratio` = 1.0: Matches real-time speed;
- 0.0 < `time_ratio` < 1.0: Slow motion at `time_ratio` times real speed;
- `time_ratio` = 0.0: Paused;
- Negative `time_ratio`: Equivalent to 0.0 (time cannot reverse);

The `time_manipulator` executor synchronizes with real time once during process startup and executor initialization, then maintains independent time progression based on the time ratio. As documented in [Executor](../interface_cpp/executor.md), the time ratio mainly affects the executor's `Now` method return value and timing for `ExecuteAt`/`ExecuteAfter` methods.

Here's a simple example:
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

The {{ '[time_manipulator.proto]({}/src/protocols/plugins/time_manipulator_plugin/time_manipulator.proto)'.format(code_site_root_path_url) }} protocol file defines a `TimeManipulatorService` with these interfaces:
- **SetTimeRatio**: Set time ratio;
- **Pause**: Pause;
- **GetTimeRatio**: Get current time ratio;



### SetTimeRatio

The `SetTimeRatio` interface sets the time ratio for a `time_manipulator` executor, defined as:
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

Here's an example using curl with the http RPC backend from **net_plugin**:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.time_manipulator_plugin.TimeManipulatorService/SetTimeRatio' \
    -d '{"executor_name": "time_schedule_executor", "time_ratio": 2.0}'
```

This command sets the time ratio to 2.0 for the `time_schedule_executor`. If successful, the command returns:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 34

{"time_ratio":2,"code":0,"msg":""}
```

### Pause

The `Pause` interface pauses a `time_manipulator` executor, defined as:
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

Here's an example using curl with the http RPC backend from **net_plugin**:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.time_manipulator_plugin.TimeManipulatorService/Pause' \
    -d '{"executor_name": "time_schedule_executor"}'
```

This command pauses the `time_schedule_executor`. If successful, it returns:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 34

{"time_ratio":0,"code":0,"msg":""}
```

### GetTimeRatio

The `GetTimeRatio` interface retrieves the current time ratio of a `time_manipulator` executor, defined as:
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

Here's an example using curl with the http RPC backend from **net_plugin**:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.time_manipulator_plugin.TimeManipulatorService/GetTimeRatio' \
    -d '{"executor_name": "time_schedule_executor"}'
```

This command retrieves the current time ratio of `time_schedule_executor`. If successful, it returns:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 34

{"time_ratio":1,"code":0,"msg":""}
```