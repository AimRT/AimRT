# Logger

## 相关链接

参考示例：
- {{ '[helloworld_module.py]({}/src/examples/py/helloworld/helloworld_module.py)'.format(code_site_root_path_url) }}


## 接口概述

`aimrt_py`包提供了以下接口来打印日志：
- `trace(logger, msg)`
- `debug(logger, msg)`
- `info(logger, msg)`
- `warn(logger, msg)`
- `error(logger, msg)`
- `fatal(logger, msg)`

这些接口的第一个参数是一个`LoggerRef`句柄类型，第二个参数是日志字符串。模块可以通过调用`CoreRef`句柄的`GetLogger()`接口，获取`LoggerRef`句柄。


## 使用示例

模块开发者可以直接参照以下示例的方式，使用分配给模块的日志句柄来打印日志：
```python
import aimrt_py

class HelloWorldModule(aimrt_py.ModuleBase):
    def Initialize(self, core):
        # Get log handle
        logger = core.GetLogger()

        # Print log
        aimrt_py.trace(logger, "This is a test trace log")
        aimrt_py.debug(logger, "This is a test debug log")
        aimrt_py.info(logger, "This is a test info log")
        aimrt_py.warn(logger, "This is a test warn log")
        aimrt_py.error(logger, "This is a test error log")
        aimrt_py.fatal(logger, "This is a test fatal log")

        return True
```
