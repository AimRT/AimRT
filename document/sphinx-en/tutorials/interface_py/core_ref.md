# CoreRef

## Related Links

Reference example:
- {{ '[examples_py_helloworld_app_mode.py]({}/src/examples/py/helloworld/examples_py_helloworld_app_mode.py)'.format(code_site_root_path_url) }}


## Interface Overview

`CoreRef` is the root handle type for invoking framework functionality. It can be obtained in two ways:
- When a developer inherits the `ModuleBase` type to implement their own `Module`, the AimRT framework passes a `CoreRef` handle in the `Initialize` method;
- When using the Create Module approach, AimRT returns the corresponding module's `CoreRef` handle after creating a `Module`;

The core interfaces provided in `CoreRef` are as follows:
- `Info()->ModuleInfo` : Get module information;
- `GetConfigurator()->ConfiguratorRef` : Get configuration handle;
- `GetLogger()->LoggerRef` : Get logger handle;
- `GetExecutorManager()->ExecutorManagerRef` : Get executor handle;
- `GetRpcHandle()->RpcHandleRef` : Get RPC handle;
- `GetChannelHandle()->ChannelHandleRef` : Get Channel handle;

Key points regarding the use of `CoreRef`:
- The AimRT framework generates a dedicated `CoreRef` handle for each module to enable features like resource isolation and monitoring. The module it belongs to can be identified via the `CoreRef::Info` interface.
- Corresponding component handles can be obtained through the `GetXXX` interfaces in `CoreRef` to invoke related functionalities. Refer to the documentation for specific components:
  - [Configurator](./configurator.md)
  - [Executor](./executor.md)
  - [Logger](./logger.md)
  - [Channel](./channel.md)
  - [Rpc](./rpc.md)

## Usage Examples

Below is a simple usage example demonstrating how to obtain and use the `CoreRef` handle when inheriting the `ModuleBase` type:

```python
import aimrt_py

class HelloWorldModule(aimrt_py.ModuleBase):
    def Initialize(self, core):
        assert(isinstance(core, aimrt_py.CoreRef))

        # Get log handle
        logger = core.GetLogger()

        # Use log handle
        aimrt_py.info(logger, "This is a test log")
        return True
```


Below is another simple usage example demonstrating how to obtain and use the `CoreRef` handle in App mode:

```python
import aimrt_py

def main():
    aimrt_core = aimrt_py.Core()

    # Initialize
    core_options = aimrt_py.CoreOptions()
    core_options.cfg_file_path = "path/to/cfg/xxx_cfg.yaml"
    aimrt_core.Initialize(core_options)

    # Get module handle
    module_handle = aimrt_core.CreateModule("HelloWorldPyModule")
    assert(isinstance(module_handle, aimrt_py.CoreRef))

    # Use log handle
    aimrt_py.info(module_handle.GetLogger(), "This is an example log.")

    # ...

if __name__ == '__main__':
    main()
```
