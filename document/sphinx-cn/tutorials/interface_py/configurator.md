# Configurator

## 相关链接

参考示例：
- {{ '[helloworld_module.py]({}/src/examples/py/helloworld/helloworld_module.py)'.format(code_site_root_path_url) }}


## 接口概述


模块可以通过调用`CoreRef`句柄的`GetConfigurator()`接口，获取`ConfiguratorRef`句柄，通过其使用一些配置相关的功能。其提供的核心接口如下：
- `GetConfigFilePath()->str` : 用于获取模块配置文件的路径。
  - 请注意，此接口仅返回一个模块配置文件的路径，模块开发者需要自己读取配置文件并解析。
  - 这个接口具体会返回什么样的路径，请参考部署运行阶段[aimrt.module 配置文档](../cfg/module.md)。


## 使用示例

一个简单的使用示例如下：
```python
import aimrt_py
import yaml

class HelloWorldModule(aimrt_py.ModuleBase):
    def Initialize(self, core):
        assert(isinstance(core, aimrt_py.CoreRef))

        # Get configurator
        configurator = core.GetConfigurator()
        assert(isinstance(configurator, aimrt_py.ConfiguratorRef))

        # Get cfg file path
        cfg_file_path = configurator.GetConfigFilePath()

        # Resolve the configuration file based on the file format actually used by the user. In this example, the analysis is based on YAML
        with open(cfg_file_path, 'r') as file:
            data = yaml.safe_load(file)
            # ...

        return True
```
