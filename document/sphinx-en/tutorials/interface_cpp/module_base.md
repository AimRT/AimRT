# ModuleBase


## Related Links

Code file:
- {{ '[aimrt_module_cpp_interface/module_base.h]({}/src/interface/aimrt_module_cpp_interface/module_base.h)'.format(code_site_root_path_url) }}

Reference example:
- {{ '[helloworld_module.cc]({}/src/examples/cpp/helloworld/module/helloworld_module/helloworld_module.cc)'.format(code_site_root_path_url) }}


## Interface Overview

The `ModuleBase` type is a base class for modules. Developers can inherit from `ModuleBase` to implement their own `Module`. It defines several interfaces that business modules need to implement:


```cpp
namespace aimrt {

class ModuleBase {
 public:
  virtual ModuleInfo Info() const = 0;

  virtual bool Initialize(CoreRef core) = 0;

  virtual bool Start() = 0;

  virtual void Shutdown() = 0;
};

}  // namespace aimrt
```


The `aimrt::ModuleInfo` structure is declared as follows:

```cpp
namespace aimrt {

struct ModuleInfo {
  std::string_view name;  // Require

  uint32_t major_version = 0;
  uint32_t minor_version = 0;
  uint32_t patch_version = 0;
  uint32_t build_version = 0;

  std::string_view author;
  std::string_view description;
};

}  // namespace aimrt
```


Regarding these virtual interfaces, the descriptions are as follows:
- `ModuleInfo Info()`: Used by AimRT to obtain module information, including module name, module version, etc.;
  - AimRT calls the `Info` interface when loading the module to read module information;
  - In the `ModuleInfo` structure, except for `name` which is required, the rest are optional;
  - If the module throws an exception in this interface, it is equivalent to returning an empty ModuleInfo;
- `bool Initialize(CoreRef)`: Used to initialize the module;
  - During the Initialize phase, AimRT calls the `Initialize` method of each module in sequence;
  - AimRT calls the module's `Initialize` method in the main thread, and the module should not block the `Initialize` method for too long;
  - When AimRT calls the module's `Initialize` method, it passes in a CoreRef handle. The module can store this handle and use it to call framework functions later;
  - Before AimRT calls the module's `Initialize` method, all components (such as configuration, logging, etc.) have completed Initialize but have not yet started;
  - If the module throws an exception in the `Initialize` method, it is equivalent to returning false;
  - If any module returns false when AimRT calls its `Initialize` method, the entire AimRT will fail to Initialize and exit;
- `bool Start()`: Used to start the module;
  - During the Start phase, AimRT calls the `Start` method of each module in sequence;
  - AimRT calls the module's `Start` method in the main thread, and the module should not block the `Start` method for too long;
  - Before AimRT calls the module's `Start` method, all components (such as configuration, logging, etc.) have entered the Start phase;
  - If the module throws an exception in the `Start` method, it is equivalent to returning false;
  - If any module returns false when AimRT calls its `Start` method, the entire AimRT will fail to Start and exit;
- `void Shutdown()`: Used to stop the module, generally for graceful exit of the entire process;
  - During the Shutdown phase, AimRT calls the `Shutdown` method of each module in sequence;
  - AimRT calls the module's `Shutdown` method in the main thread, and the module should not block the `Shutdown` method for too long;
  - AimRT may enter the Shutdown phase directly at any stage;
  - If the module throws an exception in the `Shutdown` method, AimRT will catch it and return directly;
  - After AimRT calls the module's `Shutdown` method, various components (such as configuration, logging, etc.) will then Shutdown;


## Usage Example

Below is a simple example implementing a basic HelloWorld module:

```cpp
#include "aimrt_module_cpp_interface/module_base.h"

class HelloWorldModule : public aimrt::ModuleBase {
 public:
  HelloWorldModule() = default;
  ~HelloWorldModule() override = default;

  ModuleInfo Info() const override {
    return ModuleInfo{.name = "HelloWorldModule"};
  }

  bool Initialize(aimrt::CoreRef core) override { return true; }

  bool Start() override { return true; }

  void Shutdown() override {}
};
```
