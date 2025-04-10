

# aimrt.executor

## Configuration Overview

The `aimrt.executor` configuration item is used to configure executors. Detailed configuration item descriptions are as follows:

| Node                  | Type      | Optional | Default | Description |
| ----                  | ----      | ----     | ----    | ----        |
| executors             | array     | Optional | []      | List of executors |
| executors[i].name     | string    | Required | ""      | Executor name |
| executors[i].type     | string    | Required | ""      | Executor type |
| executors[i].options  | map       | Optional | -       | Specific executor configurations |

Configuration notes for `aimrt.executor`:
- `executors` is an array used to configure various executors.
  - `executors[i].name` represents the executor name. Duplicate names are not allowed.
  - `executors[i].type` represents the executor type. Agibot officially provides several executor types, with some plugins offering additional types.
  - `executors[i].options` contains initialization parameters passed to each executor. The configuration format is defined by each executor type. Refer to corresponding documentation chapters.

Example:
```yaml
aimrt:
  executor:
    executors:
      - name: xxx_executor
        type: asio_thread
        options:
          thread_num: 2
      - name: yyy_executor
        type: tbb_thread
        options:
          thread_num: 1
```

## simple_thread Executor

The `simple_thread` executor is a simple single-thread executor that does not support scheduled tasks. Its configuration items are:

| Node                  | Type               | Optional | Default | Description |
| ----                  | ----               | ----     | ----    | ----        |
| thread_sched_policy   | string             | Optional | ""      | Thread scheduling policy |
| thread_bind_cpu       | unsigned int array | Optional | []      | CPU core binding configuration |
| queue_threshold       | unsigned int       | Optional | 10000   | Task queue threshold |

Usage notes:
- `thread_sched_policy` and `thread_bind_cpu` refer to thread binding configurations in [Common Information](./common.md)
- `queue_threshold` sets the maximum queued tasks. New tasks will fail when exceeding this threshold.

Example:
```yaml
aimrt:
  executor:
    executors:
      - name: test_simple_thread_executor
        type: simple_thread
        options:
          thread_sched_policy: SCHED_FIFO:80
          thread_bind_cpu: [0, 1]
          queue_threshold: 10000
```

## asio_thread Executor

The `asio_thread` executor is implemented using the [Asio library](https://github.com/chriskohlhoff/asio), supporting thread pools and scheduled tasks. Configuration items:

| Node                          | Type               | Optional | Default  | Description |
| ----                          | ----               | ----     | ----     | ----        |
| thread_num                    | unsigned int       | Optional | 1        | Number of threads |
| thread_sched_policy           | string             | Optional | ""       | Thread scheduling policy |
| thread_bind_cpu               | unsigned int array | Optional | []       | CPU core binding |
| timeout_alarm_threshold_us    | unsigned int       | Optional | 1000000  | Scheduling timeout threshold (μs) |
| use_system_clock              | bool               | Optional | false    | Use std::system_clock |

Usage notes:
- Thread-safe when `thread_num`=1
- `timeout_alarm_threshold_us` triggers warnings when task execution exceeds specified time
- System clock usage affects time synchronization

Example:
```yaml
aimrt:
  executor:
    executors:
      - name: test_asio_thread_executor
        type: asio_thread
        options:
          thread_num: 2
          thread_sched_policy: SCHED_FIFO:80
          thread_bind_cpu: [0, 1]
          timeout_alarm_threshold_us: 1000
          use_system_clock: false
```

## asio_strand Executor

The `asio_strand` executor is a pseudo-executor dependent on `asio_thread`, implemented using Asio's strand. Configuration items:

| Node                              | Type              | Optional | Default  | Description |
| ----                              | ----              | ----     | ----     | ----        |
| bind_asio_thread_executor_name    | string            | Required | ""       | Bound asio_thread name |
| timeout_alarm_threshold_us        | unsigned int      | Optional | 1000000  | Timeout threshold (μs) |
| use_system_clock                  | bool              | Optional | false    | Use std::system_clock |

Usage notes:
- Requires binding to existing `asio_thread` executor
- Inherits thread safety characteristics from bound executor

Example:
```yaml
aimrt:
  executor:
    executors:
      - name: test_asio_thread_executor
        type: asio_thread
        options:
          thread_num: 2
      - name: test_asio_strand_executor
        type: asio_strand
        options:
          bind_asio_thread_executor_name: test_asio_thread_executor
          timeout_alarm_threshold_us: 1000
          use_system_clock: false
```

## tbb_thread Executor

The `tbb_thread` executor uses [oneTBB](https://github.com/oneapi-src/oneTBB) for lock-free concurrency. Configuration items:

| Node                          | Type               | Optional | Default  | Description |
| ----                          | ----               | ----     | ----     | ----        |
| thread_num                    | unsigned int       | Optional | 1        | Number of threads |
| thread_sched_policy           | string             | Optional | ""       | Thread scheduling policy |
| thread_bind_cpu               | unsigned int array | Optional | []       | CPU core binding |
| timeout_alarm_threshold_us    | unsigned int       | Optional | 1000000  | Timeout threshold (μs) |
| queue_threshold               | unsigned int       | Optional | 10000    | Task queue threshold |

Usage notes:
- Lock-free implementation with configurable thread pool
- Queue threshold limits task queuing

Example:
```yaml
aimrt:
  executor:
    executors:
      - name: test_tbb_thread_executor
        type: tbb_thread
        options:
          thread_num: 2
          thread_sched_policy: SCHED_FIFO:80
          thread_bind_cpu: [0, 1]
          timeout_alarm_threshold_us: 1000
          queue_threshold: 10000
```


## time_wheel 执行器

The `time_wheel` executor is an executor based on the time wheel algorithm, typically used in scenarios with a large number of timed tasks that don't require high timing precision, such as RPC timeout handling. It starts a separate thread to run the time wheel and also supports delivering specific tasks to other executors for execution. All its configuration items are as follows:

| Node                  | Type                  | Optional | Default  | Description |
| ----                  | ----                  | ----     | ----     | ----        |
| bind_executor         | string                | Optional | ""       | Bound executor |
| dt_us                 | unsigned int          | Optional | 100000   | Time wheel tick interval in microseconds |
| wheel_size            | unsigned int array    | Optional | [100, 360] | Sizes of each time wheel |
| thread_sched_policy   | string                | Optional | ""       | Time wheel thread scheduling policy |
| thread_bind_cpu       | unsigned int array    | Optional | []       | Time wheel thread CPU binding |
| use_system_clock      | bool                  | Optional | false    | Whether to use std::system_clock, default uses std::steady_clock |

Usage notes:
- `bind_executor` is used to configure the bound executor, which will execute tasks when the time is reached.
  - If no other executor is bound, all tasks will be executed in the time wheel thread, which may block the time wheel's tick.
  - If no other executor is bound, this executor is thread-safe. If bound to another executor, thread safety depends on the bound executor.
  - If the bound executor does not exist, an exception will be thrown during initialization.
- `dt_us` is a parameter of the time wheel algorithm, representing the tick interval. Larger intervals result in lower timing precision but save CPU resources.
- `wheel_size` is another parameter of the time wheel algorithm, representing the sizes of each time wheel. For example, the default parameter `[1000, 600]` means there are two time wheels, the first with 1000 ticks and the second with 600 ticks. If the tick time is 1ms, the first wheel's full cycle is 1s, and the second wheel's full cycle is 10min. Generally, it's best to have all possible timing durations fall within the wheel.
- `thread_sched_policy` and `thread_bind_cpu` refer to the thread binding configuration in [Common Information](./common.md).
- `use_system_clock` configures whether to use std::system_clock as the time system, defaulting to false and using std::steady_clock. Note that when using std::system_clock, the executor's time will synchronize with the system and may be affected by external adjustments.

Here is a simple example:
```yaml
aimrt:
  executor:
    executors:
      - name: test_tbb_thread_executor
        type: tbb_thread
        options:
          thread_num: 2
      - name: test_time_wheel_executor
        type: time_wheel
        options:
          bind_executor: test_tbb_thread_executor
          dt_us: 1000
          wheel_size: [1000, 600]
          thread_sched_policy: SCHED_FIFO:80
          thread_bind_cpu: [0]
```