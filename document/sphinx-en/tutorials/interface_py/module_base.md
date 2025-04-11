

# ModuleBase

## Related Links

Reference example:
- {{ '[helloworld_module.py]({}/src/examples/py/helloworld/helloworld_module.py)'.format(code_site_root_path_url) }}

## Interface Overview

The `ModuleBase` type serves as a base class for modules. Developers can inherit from `ModuleBase` to implement their own modules. It defines several essential interfaces for business modules:

- `Info()->ModuleInfo`: Used by AimRT to obtain module information including name, version, etc.
  - AimRT calls the `Info` interface during module loading to read metadata
  - Only the `name` field is mandatory in the `ModuleInfo` structure, others are optional
  - Any exception thrown is equivalent to returning an empty ModuleInfo

- `Initialize(CoreRef)->bool`: Used for module initialization
  - AimRT sequentially calls each module's `Initialize` method during initialization phase
  - Called in main thread - avoid long blocking operations
  - Receives CoreRef handle for accessing framework features
  - All components (config, logging, etc.) have completed Initialize but not Start when called
  - Exception throwing is equivalent to returning false
  - Any module returning false causes AimRT initialization failure

- `Start()->bool`: Used to activate the module
  - AimRT sequentially calls `Start` methods during startup phase
  - Called in main thread - avoid long blocking operations
  - All components have entered Start phase before this call
  - Exception throwing is equivalent to returning false
  - Any module returning false causes AimRT startup failure

- `Shutdown()`: Used for graceful termination
  - AimRT sequentially calls `Shutdown` during termination phase
  - Called in main thread - avoid long blocking operations
  - May be triggered from any operational phase
  - Exceptions are caught and ignored
  - Component shutdown occurs after module Shutdown calls

## ModuleInfo Structure

The `ModuleInfo` type contains these members (only `name` is required):
- **name**: str
- **major_version**: int
- **minor_version**: int 
- **patch_version**: int
- **build_version**: int
- **author**: str
- **description**: str

## Usage Example

Here's a basic HelloWorld module implementation:
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