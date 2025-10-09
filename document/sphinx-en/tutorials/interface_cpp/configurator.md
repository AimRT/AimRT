# Configurator

## Related Links

Code files:
- {{ '[aimrt_module_cpp_interface/configurator/configurator.h]({}/src/interface/aimrt_module_cpp_interface/configurator/configurator.h)'.format(code_site_root_path_url) }}

Reference examples:
- {{ '[helloworld_module.cc]({}/src/examples/cpp/helloworld/module/helloworld_module/helloworld_module.cc)'.format(code_site_root_path_url) }}

## Interface Overview

The `aimrt::configurator::ConfiguratorRef` type is a configuration handle type. Modules can obtain a handle of this type by calling the `GetConfigurator()` interface of the `CoreRef` handle, and use it to access configuration-related functionality. Its core interfaces are as follows:


```cpp
namespace aimrt::configurator {

class ConfiguratorRef {
 public:
  std::string_view GetConfigFilePath() const;
};

}  // namespace aimrt::configurator
```


Usage notes:
- The `std::string_view GetConfigFilePath()` interface is used to get the path to the module configuration file.
  - Please note that this interface only returns the path to a module configuration file; module developers need to read and parse the configuration file themselves.
  - For what specific value this interface will return, please refer to the deployment runtime [aimrt.module configuration documentation](../cfg/module.md).

## Usage Example

A simple usage example is as follows:

```cpp
#include "yaml-cpp/yaml.h"

class HelloWorldModule : public aimrt::ModuleBase {
 public:
  // ...

  bool Initialize(aimrt::CoreRef core) override {
    // Get configurator handle
    auto configurator = core.GetConfigurator();

    // Get cfg path
    std::string_view cfg_file_path = configurator.GetConfigFilePath();

    // Parse cfg file based on the actual format. In this example, parsing based on yaml
    YAML::Node cfg_node = YAML::LoadFile(std::string(cfg_file_path));

    // ...

    return true;
  }
};
```
