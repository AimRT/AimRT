# Executor

## Related Links

Reference example:
- {{ '[helloworld_module.py]({}/src/examples/py/helloworld/helloworld_module.py)'.format(code_site_root_path_url) }}

## Interface Overview

For the specific concept of executors, please refer to the introduction in [Executor-CPP Interface](../interface_cpp/executor.md). This section only covers the specific syntax of executor functionality in AimRT Python. Modules can obtain the `ExecutorManagerRef` handle by calling the `GetExecutorManager()` interface of the `CoreRef` handle, which provides a simple way to get an `Executor`:
- `GetExecutor(str)->ExecutorRef`: Get an executor handle by name.

Executors in AimRT have some inherent attributes, most of which are related to the **executor type** and do not change during runtime. These inherent attributes include:
- **Executor Type**: A string field identifying the executor's type during runtime.
  - An AimRT instance may contain multiple types of executors. AimRT officially provides several executors, and plugins can also offer new types.
  - For specific executor types and their characteristics, refer to the `executor` configuration section in the deployment documentation.
  - During logical development, there is no need to focus too much on the actual runtime executor type; simply implement business logic based on the abstract executor interface.
- **Executor Name**: A string field identifying the executor's name during runtime.
  - Within an AimRT process, the name uniquely identifies an executor.
  - All executor instance names are determined at runtime through configuration. For details, refer to the `executor` configuration section in the deployment documentation.
  - The `GetExecutor` method of `ExecutorManagerRef` can be used to obtain an executor with a specified name.
- **Thread Safety**: A boolean value indicating whether the executor is thread-safe.
  - Typically related to the executor type.
  - Thread-safe executors ensure that tasks submitted to them will not run simultaneously; otherwise, this cannot be guaranteed.
- **Supports Timed Scheduling**: A boolean value indicating whether the executor supports timed scheduling interfaces, i.e., the `ExecuteAt` and `ExecuteAfter` interfaces.
  - If the executor does not support timed scheduling, calling `ExecuteAt` or `ExecuteAfter` will throw an exception.

Users can call the `GetExecutor` method in `ExecutorManagerRef` to obtain an `ExecutorRef` handle with a specified name and use executor-related functionalities. The core interfaces and usage instructions for `ExecutorRef` are as follows:
- `Type()->str`: Get the executor's type.
- `Name()->str`: Get the executor's name.
- `ThreadSafe()->bool`: Returns whether the executor is thread-safe.
- `IsInCurrentExecutor()->bool`: Determines whether the current call is within this executor.
  - Note: If it returns true, the current environment is definitely within this executor; if it returns false, the current environment may or may not be within this executor.
- `SupportTimerSchedule()->bool`: Returns whether the executor supports timed scheduling interfaces, i.e., the `ExecuteAt` and `ExecuteAfter` interfaces.
- `Execute(Task)`: Submits a task to this executor for immediate execution upon scheduling.
  - The `Task` parameter can be simply viewed as a task closure that satisfies the `()->void` signature.
  - This interface can be called during the Initialize/Start phase, but the executor only guarantees task execution after the Start phase. Therefore, calling this interface before the Start phase may only queue the task in the executor's task queue without immediate execution, which will begin after Start.
- `Now()->datetime`: Gets the current time within the executor's time system.
  - For most executors, this returns the standard system time.
  - Some special executors with time adjustment capabilities may return processed time.
- `ExecuteAt(datetime, Task)`: Executes a task at a specified time.
  - The first parameter—the time point—is based on the executor's time system.
  - The second parameter `Task` can be simply viewed as a task closure that satisfies the `()->void` signature.
  - If the executor does not support timed scheduling, calling this interface will throw an exception.
  - This interface can be called during the Initialize/Start phase, but the executor only guarantees task execution after the Start phase. Therefore, calling this interface before the Start phase may only queue the task in the executor's task queue without immediate execution, which will begin after Start.
- `ExecuteAfter(timedelta, Task)`: Executes a task after a specified duration.
  - The first parameter—the time duration—is based on the executor's time system.
  - The second parameter `Task` can be simply viewed as a task closure that satisfies the `()->void` signature.
  - If the executor does not support timed scheduling, calling this interface will throw an exception.
  - This interface can be called during the Initialize/Start phase, but the executor only guarantees task execution after the Start phase. Therefore, calling this interface before the Start phase may only queue the task in the executor's task queue without immediate execution, which will begin after Start.

## Usage Example

The following is a simple usage example demonstrating how to obtain an executor handle and submit a simple task to it for execution:
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