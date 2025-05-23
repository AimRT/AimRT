# ModuleBase

## Related Links

Code Files:
- {{ '[aimrt_module_cpp_interface/module_base.h]({}/src/interface/aimrt_module_cpp_interface/module_base.h)'.format(code_site_root_path_url) }}

Reference Examples:
- {{ '[helloworld_module.cc]({}/src/examples/cpp/helloworld/module/helloworld_module/helloworld_module.cc)'.format(code_site_root_path_url) }}

## Interface Overview

The `ModuleBase` type is a base class for modules. Developers can inherit from `ModuleBase` to implement their own `Module`. It defines several interfaces required for business modules:

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

Regarding these virtual interfaces, the explanations are as follows:
- `ModuleInfo Info()`: Used by AimRT to obtain module information, including module name, version, etc.;
  - AimRT calls the `Info` interface when loading the module to read module information;
  - In the `ModuleInfo` structure, only `name` is mandatory, while the rest are optional;
  - If the module throws an exception here, it is equivalent to returning an empty ModuleInfo;
- `bool Initialize(CoreRef)`: Used to initialize the module;
  - During the Initialize phase, AimRT sequentially calls the `Initialize` method of each module;
  - AimRT calls the module's `Initialize` method in the main thread, and the module should not block the `Initialize` method for too long;
  - When calling the module's `Initialize` method, AimRT passes a CoreRef handle, which the module can store and use later to call framework functions;
  - Before AimRT calls the module's `Initialize` method, all components (e.g., configuration, logging, etc.) have completed Initialize but have not yet started;
  - If the module throws an exception in the `Initialize` method, it is equivalent to returning false;
  - If any module returns false when AimRT calls its `Initialize` method, the entire AimRT will fail to Initialize and exit;
- `bool Start()`: Used to start the module;
  - During the Start phase, AimRT sequentially calls the `Start` method of each module;
  - AimRT calls the module's `Start` method in the main thread, and the module should not block the `Start` method for too long;
  - Before AimRT calls the module's `Start` method, all components (e.g., configuration, logging, etc.) have entered the Start phase;
  - If the module throws an exception in the `Start` method, it is equivalent to returning false;
  - If any module returns false when AimRT calls its `Start` method, the entire AimRT will fail to Start and exit;
- `void Shutdown()`: Used to stop the module, typically for graceful process exit;
  - During the Shutdown phase, AimRT sequentially calls each module's `Shutdown` method;
  - AimRT calls the module's `Shutdown` method in the main thread, and the module should not block the `Shutdown` method for too long;
  - AimRT may directly enter the Shutdown phase at any stage;
  - If the module throws an exception in the `Shutdown` method, AimRT will catch it and return directly;
  - After AimRT calls the module's `Shutdown` method, all components (e.g., configuration, logging, etc.) will Shutdown;

## Usage Example

The following is a simple example implementing a basic HelloWorld module:
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