# CoreRef

## 相关链接

参考示例：
- {{ '[examples_py_helloworld_app_mode.py]({}/src/examples/py/helloworld/examples_py_helloworld_app_mode.py)'.format(code_site_root_path_url) }}


## 接口概述

`CoreRef`是调用框架功能的根句柄类型，可以通过以下两种方式获取：
- 开发者继承`ModuleBase`类型实现自己的`Module`，在`Initialize`方法中，AimRT 框架会传入一个`CoreRef`句柄；
- 通过 Create Module 方式，AimRT 会在创建一个`Module`后返回对应模块的`CoreRef`句柄；


`CoreRef`中提供的核心接口如下：
- `Info()->ModuleInfo` : 获取模块信息；
- `GetConfigurator()->ConfiguratorRef` : 获取配置句柄；
- `GetLogger()->LoggerRef` ： 获取日志句柄；
- `GetExecutorManager()->ExecutorManagerRef` ： 获取执行器句柄；
- `GetRpcHandle()->RpcHandleRef` ： 获取RPC句柄；
- `GetChannelHandle()->ChannelHandleRef` ： 获取Channel句柄；


关于`CoreRef`的使用注意点如下：
- AimRT 框架会为每个模块生成一个专属`CoreRef`句柄，以实现资源隔离、监控等方面的功能。可以通过`CoreRef::Info`接口获取其所属的模块的信息。
- 可以通过`CoreRef`中的`GetXXX`接口获取对应组件的句柄，来调用相关功能。具体组件的文档请参考：
  - [Configurator](./configurator.md)
  - [Executor](./executor.md)
  - [Logger](./logger.md)
  - [Channel](./channel.md)
  - [Rpc](./rpc.md)


## 使用示例

以下是一个简单的使用示例，演示了继承`ModuleBase`类型时如何获取并使用`CoreRef`句柄：
```python
import aimrt_py

class HelloWorldModule(aimrt_py.ModuleBase):
    def Initialize(self, core):
        assert(isinstance(core, aimrt_py.CoreRef))

        # Get log handle
        logger = core.GetLogger()

        # Use log handle
        aimrt_py.info(logger, "This is a test log")
        return True
```



以下是另一个简单的使用示例，演示了 App 模式下如何获取并使用`CoreRef`句柄：
```python
import aimrt_py

def main():
    aimrt_core = aimrt_py.Core()

    # Initialize
    core_options = aimrt_py.CoreOptions()
    core_options.cfg_file_path = "path/to/cfg/xxx_cfg.yaml"
    aimrt_core.Initialize(core_options)

    # Get module handle
    module_handle = aimrt_core.CreateModule("HelloWorldPyModule")
    assert(isinstance(module_handle, aimrt_py.CoreRef))

    # Use log handle
    aimrt_py.info(module_handle.GetLogger(), "This is an example log.")

    # ...

if __name__ == '__main__':
    main()
```
