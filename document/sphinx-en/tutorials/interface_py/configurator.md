# Configurator

## Related Links

Reference Example:
- {{ '[helloworld_module.py]({}/src/examples/py/helloworld/helloworld_module.py)'.format(code_site_root_path_url) }}


## Interface Overview

Modules can obtain the `ConfiguratorRef` handle by calling the `GetConfigurator()` interface of the `CoreRef` handle, and use some configuration-related functions through it. The core interfaces it provides are as follows:
- `GetConfigFilePath()->str`: Used to obtain the path of the module configuration file.
  - Please note that this interface only returns the path of a module configuration file. Module developers need to read and parse the configuration file themselves.
  - For details about what path this interface will return, please refer to the [aimrt.module Configuration Document](../cfg/module.md) during the deployment and runtime phase.


## Usage Example

A simple usage example is as follows:
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