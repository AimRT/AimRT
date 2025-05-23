# ModuleBase

## Related Links

Reference example:
- {{ '[helloworld_module.py]({}/src/examples/py/helloworld/helloworld_module.py)'.format(code_site_root_path_url) }}


## Interface Overview

The `ModuleBase` type is a base class for modules. Developers can inherit from `ModuleBase` to implement their own `Module`. It defines several interfaces that business modules need to implement:
- `Info()->ModuleInfo`: Used by AimRT to obtain module information, including module name, version, etc.;
  - AimRT calls the `Info` interface when loading the module to read its information;
  - In the `ModuleInfo` structure, only `name` is mandatory, while all other fields are optional;
  - If the module throws an exception here, it is equivalent to returning an empty ModuleInfo;
- `Initialize(CoreRef)->bool`: Used to initialize the module;
  - During the Initialize phase, AimRT sequentially calls the `Initialize` method of each module;
  - AimRT calls the module's `Initialize` method in the main thread, so the module should not block this method for too long;
  - When calling the module's `Initialize` method, AimRT passes a CoreRef handle, which the module can store and use later to call framework functions;
  - Before AimRT calls the module's `Initialize` method, all components (e.g., configuration, logging) have completed Initialize but have not yet started;
  - If the module throws an exception in the `Initialize` method, it is equivalent to returning false;
  - If any module returns false when AimRT calls its `Initialize` method, the entire AimRT will fail to Initialize and exit;
- `Start()->bool`: Used to start the module;
  - During the Start phase, AimRT sequentially calls the `Start` method of each module;
  - AimRT calls the module's `Start` method in the main thread, so the module should not block this method for too long;
  - Before AimRT calls the module's `Start` method, all components (e.g., configuration, logging) have entered the Start phase;
  - If the module throws an exception in the `Start` method, it is equivalent to returning false;
  - If any module returns false when AimRT calls its `Start` method, the entire AimRT will fail to Start and exit;
- `Shutdown()`: Used to stop the module, typically for graceful process termination;
  - During the Shutdown phase, AimRT sequentially calls the `Shutdown` method of each module;
  - AimRT calls the module's `Shutdown` method in the main thread, so the module should not block this method for too long;
  - AimRT may directly enter the Shutdown phase at any stage;
  - If the module throws an exception in the `Shutdown` method, AimRT will catch it and return directly;
  - After AimRT calls the module's `Shutdown` method, all components (e.g., configuration, logging) will then Shutdown;

The `ModuleInfo` type contains the following members. Except for `name`, which is mandatory, all others are optional:
- **name**: str
- **major_version**: int
- **minor_version**: int
- **patch_version**: int
- **build_version**: int
- **author**: str
- **description**: str

## Usage Example

Here is a simple example implementing a basic HelloWorld module:
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