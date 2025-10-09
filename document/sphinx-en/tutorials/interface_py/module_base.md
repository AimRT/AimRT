# ModuleBase

## Related Links

Reference example:
- {{ '[helloworld_module.py]({}/src/examples/py/helloworld/helloworld_module.py)'.format(code_site_root_path_url) }}


## Interface Overview

The `ModuleBase` type is a base class for modules. Developers can inherit from `ModuleBase` to implement their own `Module`. It defines several interfaces that business modules need to implement:
- `Info()->ModuleInfo`: Used by AimRT to obtain module information, including module name, module version, etc.;
  - AimRT will call the `Info` interface when loading the module to read module information;
  - In the `ModuleInfo` structure, except for `name` which is required, the rest are optional;
  - If the module throws an exception in this interface, it is equivalent to returning an empty ModuleInfo;
- `Initialize(CoreRef)->bool`: Used to initialize the module;
  - During the Initialize phase, AimRT will call the `Initialize` method of each module in sequence;
  - AimRT will call the module's `Initialize` method in the main thread, and the module should not block the `Initialize` method for too long;
  - When AimRT calls the module's `Initialize` method, it will pass in a CoreRef handle. The module can store this handle and use it to call framework functions later;
  - Before AimRT calls the module's `Initialize` method, all components (such as configuration, logging, etc.) have completed Initialize but have not yet started;
  - If the module throws an exception in the `Initialize` method, it is equivalent to returning false;
  - If any module returns false when AimRT calls its `Initialize` method, the entire AimRT will fail to Initialize and exit;
- `Start()->bool`: Used to start the module;
  - AimRT will call the `Start` method of each module in sequence during the Start phase;
  - AimRT will call the module's `Start` method in the main thread, and the module should not block the `Start` method for too long;
  - Before AimRT calls the module's `Start` method, all components (such as configuration, logging, etc.) have entered the Start phase;
  - If the module throws an exception in the `Start` method, it is equivalent to returning false;
  - If any module returns false when AimRT calls its `Start` method, the entire AimRT will fail to Start and exit;
- `Shutdown()`: Used to stop the module, generally for graceful exit of the entire process;
  - AimRT will call the `Shutdown` method of each module in sequence during the Shutdown phase;
  - AimRT will call the module's `Shutdown` method in the main thread, and the module should not block the `Shutdown` method for too long;
  - AimRT may enter the Shutdown phase directly at any stage;
  - If the module throws an exception in the `Shutdown` method, AimRT will catch it and return directly;
  - After AimRT calls the module's `Shutdown` method, each component (such as configuration, logging, etc.) will then Shutdown;



The `ModuleInfo` type has the following members, where except for `name` which is required, the rest are optional:
- **name**: str
- **major_version**: int
- **minor_version**: int
- **patch_version**: int
- **build_version**: int
- **author**: str
- **description**: str

## Usage Example

The following is a simple example implementing a basic HelloWorld module:

```python
import aimrt_py

class HelloWorldModule(aimrt_py.ModuleBase):
    def __init__(self):
        super().__init__()

    def Info(self):
        info = aimrt_py.ModuleInfo()
        info.name = "HelloWorldModule"
        return info

    def Initialize(self, core):
        print("Initialize")
        return True

    def Start(self):
        print("Start")
        return True

    def Shutdown(self):
        print("Shutdown")
```
