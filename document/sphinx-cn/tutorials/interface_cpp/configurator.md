# Configurator

## 相关链接

代码文件：
- {{ '[aimrt_module_cpp_interface/configurator/configurator.h]({}/src/interface/aimrt_module_cpp_interface/configurator/configurator.h)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[helloworld_module.cc]({}/src/examples/cpp/helloworld/module/helloworld_module/helloworld_module.cc)'.format(code_site_root_path_url) }}

## 接口概述

`aimrt::configurator::ConfiguratorRef`类型是一个配置句柄类型，模块可以通过调用`CoreRef`句柄的`GetConfigurator()`接口获取该类型的句柄，通过它使用一些配置相关的功能。其提供的核心接口如下：

```cpp
namespace aimrt::configurator {

class ConfiguratorRef {
 public:
  std::string_view GetConfigFilePath() const;
};

}  // namespace aimrt::configurator
```

使用注意点：
- `std::string_view GetConfigFilePath()`接口：用于获取模块配置文件的路径。
  - 请注意，此接口仅返回一个模块配置文件的路径，模块开发者需要自己读取配置文件并解析。
  - 这个接口具体会返回什么样的值，请参考部署运行阶段[aimrt.module 配置文档](../cfg/module.md)。

## 使用示例

一个简单的使用示例如下：
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
