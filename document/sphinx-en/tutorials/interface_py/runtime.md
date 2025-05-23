# Runtime Interface

## Related Links

Reference examples:
- {{ '[helloworld]({}/src/examples/py/helloworld)'.format(code_site_root_path_url) }}
  - {{ '[examples_py_helloworld_app_mode.py]({}/src/examples/py/helloworld/examples_py_helloworld_app_mode.py)'.format(code_site_root_path_url) }}
  - {{ '[examples_py_helloworld_registration_mode.py]({}/src/examples/py/helloworld/examples_py_helloworld_registration_mode.py)'.format(code_site_root_path_url) }}

## Introduction

Unlike the CPP interface, the AimRT Python interface only provides **App Mode**. Developers need to manage the main method in Python themselves, where they create and manage the AimRT Core instance. In **App Mode**, the AimRT Python interface is similar to the CPP interface, offering two approaches for developing user module logic: **Registration** or **Creation**.

## AimRT Core

The `Core` type in the `aimrt_py` package is used to control the operation of the AimRT instance. This type provides the following key methods:
- `Initialize(core_options)`: Initializes the AimRT runtime;
- `Start()`: Starts the AimRT runtime. Note that this method blocks the current thread until the `Shutdown` method is called in another thread;
- `Shutdown()`: Stops the AimRT runtime and supports reentrancy;
- `RegisterModule(module)`: Registers a module;
- `CreateModule(module_name)->module_handle`: Creates a module;

The first three methods are runtime control methods for the AimRT instance, while the last two correspond to the **Registration** and **Creation** approaches for developing user module logic. The `Initialize` method takes a `CoreOptions` type as a parameter, which includes the following member:
- `cfg_file_path`: str, the configuration file path.

Here is a simple example that starts an AimRT runtime without loading any business logic:
```python
import threading
import time
import aimrt_py

def main():
    aimrt_core = aimrt_py.Core()

    # init
    core_options = aimrt_py.CoreOptions()
    core_options.cfg_file_path = "path/to/cfg/xxx_cfg.yaml"
    aimrt_core.Initialize(core_options)

    # start
    thread = threading.Thread(target=aimrt_core.Start)
    thread.start()

    # shutdown
    time.sleep(1)
    aimrt_core.Shutdown()

    thread.join()

if __name__ == '__main__':
    main()
```

## Registering Modules

In registration mode, developers need to inherit the `ModuleBase` base class to create their own `Module` and implement methods such as `Initialize` and `Start`. Then, they register this `Module` with the `Core` instance before calling the `Initialize` method. This approach maintains a clear `Module` boundary.

For more information about the `ModuleBase` base class, please refer to the [ModuleBase](./module_base.md) documentation.

Here is a simple example demonstrating how to write your own module and register it with the `Core` instance:
```python
import threading
import signal
import sys
import aimrt_py
import yaml

# 继承 aimrt_py.ModuleBase 实现一个Module
class HelloWorldModule(aimrt_py.ModuleBase):
    def __init__(self):
        super().__init__()
        self.core = aimrt_py.CoreRef()
        self.logger = aimrt_py.LoggerRef()

    def Info(self):
        info = aimrt_py.ModuleInfo()
        info.name = "HelloWorldModule"
        return info

    def Initialize(self, core):
        assert(isinstance(core, aimrt_py.CoreRef))

        self.core = core
        self.logger = self.core.GetLogger()

        # log
        aimrt_py.info(self.logger, "Module initialize")

        return True

    def Start(self):
        aimrt_py.info(self.logger, "Module start")

        return True

    def Shutdown(self):
        aimrt_py.info(self.logger, "Module shutdown")


global_aimrt_core = None

def signal_handler(sig, frame):
    global global_aimrt_core
    if (global_aimrt_core and (sig == signal.SIGINT or sig == signal.SIGTERM)):
        global_aimrt_core.Shutdown()
        return

    sys.exit(0)


def main():
    # 注册ctrl-c信号
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("AimRT start.")

    # 创建 aimrt_py.Core 实例
    aimrt_core = aimrt_py.Core()

    global global_aimrt_core
    global_aimrt_core = aimrt_core

    # 注册模块
    module = HelloWorldModule()
    aimrt_core.RegisterModule(module)

    # 初始化 aimrt_py.Core 实例
    core_options = aimrt_py.CoreOptions()
    core_options.cfg_file_path = "path/to/cfg/xxx_cfg.yaml"
    aimrt_core.Initialize(core_options)

    # 启动 aimrt_py.Core 实例
    thread = threading.Thread(target=aimrt_core.Start)
    thread.start()

    # 等待停止
    while thread.is_alive():
        thread.join(1.0)

    aimrt_core.Shutdown()

    global_aimrt_core = None

    print("AimRT exit.")

if __name__ == '__main__':
    main()
```

## Creating Modules

After the `Core` instance calls the `Initialize` method, a module can be created using `CreateModule`, which returns a handle. Developers can directly use this handle to call framework methods, such as RPC or Log. This approach does not maintain a clear `Module` boundary and is typically used for quickly developing small tools.

Here is a simple example of what developers need to write in a Python file:

```python
import argparse
import threading
import time
import aimrt_py
import yaml

def main():
    aimrt_core = aimrt_py.Core()

    core_options = aimrt_py.CoreOptions()
    core_options.cfg_file_path = "path/to/cfg/xxx_cfg.yaml"
    aimrt_core.Initialize(core_options)

    # create module handle
    module_handle = aimrt_core.CreateModule("HelloWorldModule")
    assert(isinstance(module_handle, aimrt_py.CoreRef))

    # use cfg and log
    module_cfg_file_path = module_handle.GetConfigurator().GetConfigFilePath()
    with open(module_cfg_file_path, 'r') as file:
        data = yaml.safe_load(file)
        aimrt_py.info(module_handle.GetLogger(), str(data))

    # start
    thread = threading.Thread(target=aimrt_core.Start)
    thread.start()
    time.sleep(1)

    # use log
    count = 0
    while(count < 10):
        count = count + 1
        aimrt_py.info(module_handle.GetLogger(), "Conut : {}.".format(count))

    # shutdown
    time.sleep(1)
    aimrt_core.Shutdown()

    thread.join()

if __name__ == '__main__':
    main()
```