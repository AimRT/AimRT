# Executor

## Related Links

Reference example:
- {{ '[helloworld_module.py]({}/src/examples/py/helloworld/helloworld_module.py)'.format(code_site_root_path_url) }}


## Interface Overview

For the specific concept of the executor, please refer to the introduction in [Executor - CPP Interface](../interface_cpp/executor.md). This section only describes the concrete syntax of the executor functionality in AimRT Python. A module can obtain an `ExecutorManagerRef` handle by calling the `GetExecutorManager()` interface of the `CoreRef` handle, which provides a simple interface to get an `Executor`:
- `GetExecutor(str)->ExecutorRef`: Obtain an executor handle by name;



Executors in AimRT have some inherent properties, most of which are related to the **executor type** and do not change during runtime. These inherent properties include:
- **Executor Type**: A string field identifying the type of the executor at runtime.
  - Within a single AimRT instance, there can be multiple types of executors. AimRT officially provides several executors, and plugins can also provide new executor types.
  - For details on executor types and their characteristics, please refer to the `executor` configuration section in the deployment phase.
  - During logical development, you should not focus too much on the actual runtime executor type; simply implement business logic according to the abstract executor interface.
- **Executor Name**: A string field identifying the name of the executor at runtime.
  - Within a single AimRT process, the name uniquely identifies an executor.
  - All executor instance names are determined at runtime through configuration. For details, please refer to the `executor` configuration section in the deployment phase.
  - You can obtain an executor with a specified name via the `GetExecutor` method of `ExecutorManagerRef`.
- **Thread Safety**: A boolean value indicating whether this executor is thread-safe.
  - Usually related to the executor type.
  - A thread-safe executor guarantees that tasks submitted to it will not run concurrently; otherwise, no such guarantee is provided.
- **Support for Time-based Scheduling**: A boolean value indicating whether this executor supports time-based scheduling interfaces, namely the `ExecuteAt` and `ExecuteAfter` interfaces.
  - If this executor does not support time-based scheduling, calling the `ExecuteAt` or `ExecuteAfter` interfaces will throw an exception.



Users can call the `GetExecutor` method in `ExecutorManagerRef` to obtain an `ExecutorRef` handle with a specified name to use executor-related functionality. The core interfaces and usage instructions for `ExecutorRef` are as follows:
- `Type()->str`: Get the type of the executor.
- `Name()->str`: Get the name of the executor.
- `ThreadSafe()->bool`: Return whether this executor is thread-safe.
- `IsInCurrentExecutor()->bool`: Determine whether the current environment is within this executor when this function is called.
  - Note: If it returns true, the current environment is definitely within this executor; if it returns false, the current environment may or may not be within this executor.
- `SupportTimerSchedule()->bool`: Return whether this executor supports time-based scheduling interfaces, namely the `ExecuteAt` and `ExecuteAfter` interfaces.
- `Execute(Task)`: Submit a task to this executor for immediate execution upon scheduling.
  - The parameter `Task` can be simply regarded as a task closure with the signature `()->void`.
  - This interface can be called during the Initialize/Start phase, but the executor only guarantees to start executing after the Start phase. Therefore, calling this interface before the Start phase may only enqueue the task into the executor's task queue without immediate execution, and the task will start executing only after the Start phase.
- `Now()->datetime`: Get the time in this executor's time system.
  - For general executors, this returns the standard system time.
  - For some special executors with time-scaling functionality, this may return processed time.
- `ExecuteAt(datetime, Task)`: Execute a task at a specific time point.
  - The first parameter - time point, is based on this executor's time system.
  - The second parameter `Task` can be simply regarded as a task closure with the signature `()->void`.
  - If this executor does not support time-based scheduling, calling this interface will throw an exception.
  - This interface can be called during the Initialize/Start phase, but the executor only guarantees to start executing after the Start phase. Therefore, calling this interface before the Start phase may only enqueue the task into the executor's task queue without immediate execution, and the task will start executing only after the Start phase.
- `ExecuteAfter(timedelta, Task)`: Execute a task after a certain duration.
  - The first parameter - duration, is based on this executor's time system.
  - The second parameter `Task` can be simply regarded as a task closure with the signature `()->void`.
  - If this executor does not support time-based scheduling, calling this interface will throw an exception.
  - This interface can be called during the Initialize/Start phase, but the executor only guarantees to start executing after the Start phase. Therefore, calling this interface before the Start phase may only enqueue the task into the executor's task queue without immediate execution, and the task will start executing only after the Start phase.


## Usage Example

Below is a simple usage example demonstrating how to obtain an executor handle and submit a simple task to the executor for execution:

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
