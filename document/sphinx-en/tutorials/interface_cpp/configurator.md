

# Configurator

## Related Links

Code Files:
- {{ '[aimrt_module_cpp_interface/configurator/configurator.h]({}/src/interface/aimrt_module_cpp_interface/configurator/configurator.h)'.format(code_site_root_path_url) }}

Reference Example:
- {{ '[helloworld_module.cc]({}/src/examples/cpp/helloworld/module/helloworld_module/helloworld_module.cc)'.format(code_site_root_path_url) }}

## Interface Overview

The `aimrt::configurator::ConfiguratorRef` type is a configuration handle. Modules can obtain this handle by calling the `GetConfigurator()` interface of the `CoreRef` handle, through which they can use configuration-related functionalities. Its core interfaces include:

```cpp
namespace aimrt::configurator {

class ConfiguratorRef {
 public:
  std::string_view GetConfigFilePath() const;
};

}  // namespace aimrt::configurator
```

Usage Notes:
- `std::string_view GetConfigFilePath()` interface: Used to get the path of the module configuration file.
  - Note that this interface only returns the path to a single module configuration file. Module developers need to read and parse the configuration file themselves.
  - For details about what exactly this interface returns, please refer to the [aimrt.module Configuration Documentation](../cfg/module.md) during the deployment phase.

## Usage Example

A simple usage example is shown below:
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