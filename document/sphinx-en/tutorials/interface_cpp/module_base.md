

# ModuleBase

## Related Links

Code Files:
- {{ '[aimrt_module_cpp_interface/module_base.h]({}/src/interface/aimrt_module_cpp_interface/module_base.h)'.format(code_site_root_path_url) }}

Reference Examples:
- {{ '[helloworld_module.cc]({}/src/examples/cpp/helloworld/module/helloworld_module/helloworld_module.cc)'.format(code_site_root_path_url) }}

## Interface Overview

The `ModuleBase` type serves as a base class for modules. Developers can inherit from `ModuleBase` to implement their own modules. It defines several essential interfaces required for business modules:

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

Explanation of these virtual interfaces:
- `ModuleInfo Info()`: Used by AimRT to obtain module information, including module name and version.
  - AimRT calls the `Info` interface during module loading to retrieve module information.
  - In the `ModuleInfo` structure, only the `name` field is mandatory; other fields are optional.
  - If the module throws an exception here, it's equivalent to returning an empty ModuleInfo.
- `bool Initialize(CoreRef)`: Used for module initialization.
  - AimRT sequentially calls each module's `Initialize` method during the initialization phase.
  - The method is called in the main thread; modules should avoid prolonged blocking in `Initialize`.
  - AimRT provides a CoreRef handle that modules can store for subsequent framework operations.
  - All components (e.g., configuration, logging) have completed initialization but not started when `Initialize` is called.
  - Throwing an exception here is equivalent to returning false.
  - If any module returns false during initialization, AimRT will fail to initialize and exit.
- `bool Start()`: Used to start the module.
  - AimRT sequentially calls each module's `Start` method during the startup phase.
  - Called in the main thread; modules should avoid prolonged blocking.
  - All components have entered the start phase before this method is called.
  - Throwing an exception here is equivalent to returning false.
  - Any module returning false will cause AimRT to fail startup and exit.
- `void Shutdown()`: Used for module shutdown, typically for graceful process termination.
  - AimRT sequentially calls each module's `Shutdown` method during termination.
  - Called in the main thread; modules should avoid prolonged blocking.
  - AimRT may enter Shutdown phase at any stage.
  - Exceptions thrown here will be caught by AimRT without propagation.
  - System components (e.g., configuration, logging) will shut down after all modules complete Shutdown.

## Usage Example

Here's a basic implementation example of a HelloWorld module:
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