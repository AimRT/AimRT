
# Runtime Interface

## 相关链接

参考示例：
- {{ '[helloworld]({}/src/examples/py/helloworld)'.format(code_site_root_path_url) }}
  - {{ '[examples_py_helloworld_app_mode.py]({}/src/examples/py/helloworld/examples_py_helloworld_app_mode.py)'.format(code_site_root_path_url) }}
  - {{ '[examples_py_helloworld_registration_mode.py]({}/src/examples/py/helloworld/examples_py_helloworld_registration_mode.py)'.format(code_site_root_path_url) }}


## 简介

与 CPP 接口不一样的是，AimRT Python 接口只提供**App模式**，开发者需要自行管理 python 中的 main 方法，并在其中创建、管理 AimRT Core 实例。在**App模式**模式下，AimRT Python 接口与 CPP 接口类似，提供了**注册**或**创建**这两种方式去开发用户的模块逻辑。


## AimRT Core

`aimrt_py`包中的`Core`类型用于控制 AimRT 实例的运行。该类型提供了以下几个关键的方法：
- `Initialize(core_options)`: 初始化 AimRT 运行时；
- `Start()`: 启动 AimRT 运行时，注意，该方法将阻塞当前线程，直到在其他线程中调用了`Shutdown`方法；
- `Shutdown()`: 停止 AimRT 运行时，支持重入；
- `RegisterModule(module)`: 注册一个模块；
- `CreateModule(module_name)->module_handle`: 创建一个模块；


前三个方法是 AimRT 实例的运行控制方法，后两个方法则对应了**注册**和**创建**这两种开发用户的模块逻辑的方式。其中，`Initialize`方法接收一个`CoreOptions`类型作为参数。此类型包含以下几个成员：
- `cfg_file_path`：str，配置文件路径


以下是一个简单的示例，该示例启动了一个 AimRT 运行时，但没有加载任何业务逻辑：
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


## 注册模块

在注册模式下，开发者需要继承`ModuleBase`基类来创建一个自己的`Module`，并实现其中的`Initialize`、`Start`等方法，然后在`Core`实例调用`Initialize`方法之前将该`Module`注册到`Core`实例中。在此方式下仍然有一个比较清晰的`Module`边界。

关于`ModuleBase`基类的相关信息，请参考[ModuleBase](./module_base.md)文档。


以下是一个简单的例子，展示了如何编写一个自己的模块，并注册到`Core`实例中：
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


## 创建模块

在`Core`实例调用`Initialize`方法之后，通过`CreateModule`可以创建一个模块，并返回一个句柄，开发者可以直接基于此句柄调用一些框架的方法，比如 RPC 或者 Log 等。在此方式下没有一个比较清晰的`Module`边界，一般仅用于快速做一些小工具。


以下是一个简单的例子，开发者需要编写的 Python 文件如下：

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

