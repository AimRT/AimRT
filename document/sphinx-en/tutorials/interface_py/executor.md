

# Executor

## Related Links

Reference examples:
- {{ '[helloworld_module.py]({}/src/examples/py/helloworld/helloworld_module.py)'.format(code_site_root_path_url) }}

## Interface Overview

For specific concepts of executors, please refer to the introduction in [Executor-CPP Interface](../interface_cpp/executor.md). This chapter only introduces the specific syntax of executor functionality in AimRT Python. Modules can obtain the `ExecutorManagerRef` handle by calling the `GetExecutorManager()` interface of the `CoreRef` handle, which provides a simple interface to obtain `Executor`:
- `GetExecutor(str)->ExecutorRef`: Get an executor handle by name.

AimRT executors have some inherent attributes that are mostly related to **executor type** and remain unchanged during runtime. These inherent attributes include:
- **Executor Type**: A string field identifying the executor's runtime type.
  - An AimRT instance may contain multiple types of executors. AimRT officially provides several executor types, and plugins can also provide new types.
  - Specific executor types and characteristics can be found in the `executor` configuration section of the deployment documentation.
  - During logic development, focus should be on abstract executor interfaces rather than actual runtime types.
- **Executor Name**: A string field uniquely identifying the executor at runtime.
  - Each executor instance's name is determined by configuration at runtime. Refer to the `executor` configuration section in deployment documentation.
  - Can be obtained via `ExecutorManagerRef`'s `GetExecutor` method.
- **Thread Safety**: A boolean value indicating whether the executor is thread-safe.
  - Typically related to executor type.
  - Thread-safe executors guarantee tasks won't run concurrently; non-thread-safe ones don't.
- **Timer Schedule Support**: A boolean value indicating whether the executor supports timed scheduling interfaces (`ExecuteAt`, `ExecuteAfter`).
  - Calling `ExecuteAt` or `ExecuteAfter` on unsupported executors will throw an exception.

Users can call the `GetExecutor` method in `ExecutorManagerRef` to obtain an `ExecutorRef` handle for specific executor operations. Core interfaces of `ExecutorRef` include:
- `Type()->str`: Get executor type.
- `Name()->str`: Get executor name.
- `ThreadSafe()->bool`: Check thread safety.
- `IsInCurrentExecutor()->bool`: Determine whether the current context is within this executor.
  - Note: True return guarantees current context is in the executor; False means it might not be.
- `SupportTimerSchedule()->bool`: Check timer scheduling support.
- `Execute(Task)`: Post a task for immediate execution.
  - `Task` can be treated as a closure with `()->void` signature.
  - Can be called during Initialize/Start phases, but execution only begins after Start phase.
- `Now()->datetime`: Get executor-system time.
  - Returns standard system time for regular executors.
  - Special executors with time scaling may return adjusted time.
- `ExecuteAt(datetime, Task)`: Schedule task at specific time.
  - Time parameter uses executor's time system.
  - Throws exception if timer scheduling unsupported.
- `ExecuteAfter(timedelta, Task)`: Schedule task after time delta.
  - Time delta uses executor's time system.
  - Throws exception if timer scheduling unsupported.

## Usage Example

The following simple example demonstrates how to obtain an executor handle and post a task to it:
```python
import aimrt_py
import datetime

class HelloWorldModule(aimrt_py.ModuleBase):
    def __init__(self):
        super().__init__()
        self.logger = aimrt_py.LoggerRef()
        self.work_executor = aimrt_py.ExecutorRef()

    def Initialize(self, core):
        assert(isinstance(core, aimrt_py.CoreRef))

        self.logger = core.GetLogger()

        # Get executor with name 'work_thread_pool'
        self.work_executor = core.GetExecutorManager().GetExecutor("work_thread_pool")
        assert(isinstance(work_executor, aimrt_py.ExecutorRef))

        # Check for executor 'work_thread_pool'
        if (not self.work_executor or not self.work_executor.SupportTimerSchedule()):
            aimrt_py.error(self.logger, "Get executor 'work_thread_pool' failed.")
            return False

        return True

    def Start(self):
        def test_task():
            aimrt_py.info(self.logger, "run test task.")

        # Deliver to the 'work_executor' and execute immediately
        self.work_executor.Execute(test_task)

        # Deliver to the 'work_executor' and execute after a period of time
        self.work_executor.ExecuteAfter(datetime.timedelta(seconds=1), test_task)

        # Deliver to the 'work_executor' and execute at a certain point in time
        self.work_executor.ExecuteAt(datetime.datetime.now() + datetime.timedelta(seconds=2), test_task)

        return True
```