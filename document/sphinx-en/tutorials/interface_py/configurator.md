

# Configurator

## Related Links

Reference examples:
- {{ '[helloworld_module.py]({}/src/examples/py/helloworld/helloworld_module.py)'.format(code_site_root_path_url) }}

## Interface Overview

Modules can obtain the `ConfiguratorRef` handle by calling the `GetConfigurator()` interface of the `CoreRef` handle, and use some configuration-related functions through it. The core interfaces it provides are as follows:
- `GetConfigFilePath()->str` : Used to get the path of the module configuration file.
  - Note that this interface only returns the path of a module configuration file. Module developers need to read and parse the configuration file themselves.
  - For details about what path this interface specifically returns, please refer to the [aimrt.module configuration document](../cfg/module.md) during deployment and runtime phase.

## Usage Example

A simple usage example is shown below:
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