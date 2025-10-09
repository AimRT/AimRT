# Configurator

## Related Links

Reference example:
- {{ '[helloworld_module.py]({}/src/examples/py/helloworld/helloworld_module.py)'.format(code_site_root_path_url) }}


## Interface Overview

A module can obtain a `ConfiguratorRef` handle by calling the `GetConfigurator()` interface of the `CoreRef` handle, and use some configuration-related features through it. The core interfaces it provides are as follows:
- `GetConfigFilePath()->str` : Used to get the path of the module configuration file.
  - Please note that this interface only returns the path of a module configuration file; the module developer needs to read and parse the configuration file themselves.
  - For what specific path this interface will return, please refer to the deployment runtime [aimrt.module configuration document](../cfg/module.md).


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
