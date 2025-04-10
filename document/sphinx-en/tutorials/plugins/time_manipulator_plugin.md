

# Time Manager Plugin

## Related Links

Protocol Files:
- {{ '[time_manipulator.proto]({}/src/protocols/plugins/time_manipulator_plugin/time_manipulator.proto)'.format(code_site_root_path_url) }}

Reference Examples:
- {{ '[time_manipulator_plugin]({}/src/examples/plugins/time_manipulator_plugin)'.format(code_site_root_path_url) }}

## Plugin Overview

The **time_manipulator_plugin** provides the `time_manipulator` executor to enable time adjustment functionality. It also registers an RPC based on protobuf protocol definitions, offering management interfaces for the `time_manipulator` executor. Note that **time_manipulator_plugin** does not provide any communication backend, so this plugin should generally be used in conjunction with other communication plugins' RPC backends, such as the http RPC backend in [net_plugin](./net_plugin.md).

Plugin configuration items:

| Node               | Type          | Optional | Default | Description |
| ------------------ | ------------- | -------- | ------- | ----------- |
| service_name       | string        | Yes      | ""      | RPC Service Name. Uses protocol-generated default if empty |

Example configuration combining **time_manipulator_plugin** with http RPC backend from **net_plugin**:

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

The `time_manipulator` executor is a speed-adjustable executor based on time wheel implementation, typically used for simulation scenarios requiring timed tasks with moderate timing precision. It runs a separate thread for the time wheel and supports task dispatching to other executors. Configuration items:

| Node                | Type                | Optional | Default      | Description |
| ------------------- | ------------------- | -------- | ------------ | ----------- |
| bind_executor       | string              | Required | ""           | Bound executor |
| dt_us               | unsigned int        | Yes      | 100000       | Time wheel tick interval (μs) |
| init_ratio          | double              | Yes      | 1.0          | Initial time ratio |
| wheel_size          | unsigned int array   | Yes      | [100, 360]   | Time wheel sizes |
| thread_sched_policy | string              | Yes      | ""           | Thread scheduling policy |
| thread_bind_cpu     | unsigned int array   | Yes      | []           | CPU affinity configuration |
| use_system_clock    | bool                | Yes      | false        | Use std::system_clock |

Key considerations:
- `bind_executor` configures the target executor for task execution
  - Must bind to an existing executor
  - Thread safety matches bound executor
  - Throws exception if bound executor doesn't exist
- `dt_us` affects timing precision and CPU usage (larger values reduce precision but save CPU)
- `init_ratio` defaults to 1.0 (real-time speed)
- `wheel_size` determines time wheel capacity. Default [100, 360] with 100μs tick provides 10ms and 3.6s ranges
- `thread_sched_policy` and `thread_bind_cpu` follow [Common Information](../cfg/common.md)
- `use_system_clock` determines clock source (system clock vs steady clock)

Time ratio (`time_ratio`) behavior:
- >1.0: Fast forward (realtime × ratio)
- 1.0: Real-time speed
- 0.0-1.0: Slow motion
- ≤0.0: Paused

The executor synchronizes with real time at initialization, then maintains independent timeline based on time ratio. Affects `Now()`, `ExecuteAt()`, and `ExecuteAfter()` methods.

Example configuration:

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

The {{ '[time_manipulator.proto]({}/src/protocols/plugins/time_manipulator_plugin/time_manipulator.proto)'.format(code_site_root_path_url) }} protocol defines `TimeManipulatorService` with interfaces:
- **SetTimeRatio**: Adjust time ratio
- **Pause**: Pause time
- **GetTimeRatio**: Retrieve current ratio

### SetTimeRatio

Interface definition:
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

Example HTTP call using curl via **net_plugin**:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.time_manipulator_plugin.TimeManipulatorService/SetTimeRatio' \
    -d '{"executor_name": "time_schedule_executor", "time_ratio": 2.0}'
```

Successful response:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 34

{"time_ratio":2,"code":0,"msg":""}
```

### Pause

Interface definition:
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

Example HTTP call:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.time_manipulator_plugin.TimeManipulatorService/Pause' \
    -d '{"executor_name": "time_schedule_executor"}'
```

Successful response:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 34

{"time_ratio":0,"code":0,"msg":""}
```

### GetTimeRatio

Interface definition:
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

Example HTTP call:
```shell
curl -i \
    -H 'content-type:application/json' \
    -X POST 'http://127.0.0.1:50080/rpc/aimrt.protocols.time_manipulator_plugin.TimeManipulatorService/GetTimeRatio' \
    -d '{"executor_name": "time_schedule_executor"}'
```

Successful response:
```
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 34

{"time_ratio":1,"code":0,"msg":""}
```