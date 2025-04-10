# Executor

## 相关链接

参考示例：
- {{ '[helloworld_module.py]({}/src/examples/py/helloworld/helloworld_module.py)'.format(code_site_root_path_url) }}


## 接口概述

执行器的具体概念请参考[执行器-CPP接口](../interface_cpp/executor.md)中的介绍。此章节仅介绍 AimRT Python 中执行器功能的具体语法。模块可以通过调用`CoreRef`句柄的`GetExecutorManager()`接口，获取`ExecutorManagerRef`句柄，其中提供了一个简单的获取`Executor`的接口：
- `GetExecutor(str)->ExecutorRef` : 通过名称获取执行器句柄；



AimRT 中的执行器有一些固有属性，这些固有属性大部分跟**执行器类型**相关，在运行过程中不会改变。这些固有属性包括：
- **执行器类型**：一个字符串字段，标识执行器在运行时的类型。
  - 在一个 AimRT 实例中，会存在多种类型的执行器，AimRT 官方提供了几种执行器，插件也可以提供新类型的执行器。
  - 具体的执行器类型以及特性请参考部署环节的`executor`配置章节。
  - 在逻辑开发过程中，不应太关注实际运行时的执行器类型，只需根据抽象的执行器接口去实现业务逻辑。
- **执行器名称**：一个字符串字段，标识执行器在运行时的名称。
  - 在一个 AimRT 进程中，名称唯一标识了一个执行器。
  - 所有的执行器实例的名称都在运行时通过配置来决定，具体请参考部署环节的`executor`配置章节。
  - 可以通过`ExecutorManagerRef`的`GetExecutor`方法，获取指定名称的执行器。
- **线程安全性**：一个 bool 值，标识了本执行器是否是线程安全的。
  - 通常和执行器类型相关。
  - 线程安全的执行器可以保证投递到其中的任务不会同时运行；反之则不能保证。
- **是否支持按时间调度**：一个 bool 值，标识了本执行器是否支持按时间调度的接口，也就是`ExecuteAt`、`ExecuteAfter`接口。
  - 如果本执行器不支持按时间调度，则调用`ExecuteAt`、`ExecuteAfter`接口时会抛出一个异常。



使用者可以调用`ExecutorManagerRef`中的`GetExecutor`方法，获取指定名称的`ExecutorRef`句柄，以调用执行器相关功能。`ExecutorRef`的核心接口和使用说明如下：
- `Type()->str`：获取执行器的类型。
- `Name()->str`：获取执行器的名称。
- `ThreadSafe()->bool`：返回本执行器是否是线程安全的。
- `IsInCurrentExecutor()->bool`：判断调用此函数时是否在本执行器中。
  - 注意：如果返回 true，则当前环境一定在本执行器中；如果返回 false，则当前环境有可能不在本执行器中，也有可能在。
- `SupportTimerSchedule()->bool`：返回本执行器是否支持按时间调度的接口，也就是`ExecuteAt`、`ExecuteAfter`接口。
- `Execute(Task)`：将一个任务投递到本执行器中，并在调度后立即执行。
  - 可将参数`Task`简单的视为一个满足`()->void`签名的任务闭包。
  - 此接口可以在 Initialize/Start 阶段调用，但执行器在 Start 阶段后才能保证开始执行，因此在 Start 阶段之前调用此接口，有可能只能将任务投递到执行器的任务队列中而暂时不执行，等到 Start 之后才开始执行任务。
- `Now()->datetime`：获取本执行器体系下的时间。
  - 对于一般的执行器来说，此处返回的都是标准的系统时间。
  - 有一些带时间调速功能的特殊执行器，此处可能会返回经过处理的时间。
- `ExecuteAt(datetime, Task)`：在某个时间点执行一个任务。
  - 第一个参数-时间点，以本执行器的时间体系为准。
  - 可将第二个参数`Task`简单的视为一个满足`()->void`签名的任务闭包。
  - 如果本执行器不支持按时间调度，则调用此接口时会抛出一个异常。
  - 此接口可以在 Initialize/Start 阶段调用，但执行器在 Start 阶段后才能保证开始执行，因此在 Start 阶段之前调用此接口，有可能只能将任务投递到执行器的任务队列中而暂时不执行，等到 Start 之后才开始执行任务。
- `ExecuteAfter(timedelta, Task)`：在某个时间后执行一个任务。
  - 第一个参数-时间段，以本执行器的时间体系为准。
  - 可将第二个参数`Task`简单的视为一个满足`()->void`签名的任务闭包。
  - 如果本执行器不支持按时间调度，则调用此接口时会抛出一个异常。
  - 此接口可以在 Initialize/Start 阶段调用，但执行器在 Start 阶段后才能保证开始执行，因此在 Start 阶段之前调用此接口，有可能只能将任务投递到执行器的任务队列中而暂时不执行，等到 Start 之后才开始执行任务


## 使用示例

以下是一个简单的使用示例，演示了如何获取一个执行器句柄，并将一个简单的任务投递到该执行器中执行：
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

