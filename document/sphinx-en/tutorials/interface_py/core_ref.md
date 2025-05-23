# CoreRef

## Related Links

Reference Example:
- {{ '[examples_py_helloworld_app_mode.py]({}/src/examples/py/helloworld/examples_py_helloworld_app_mode.py)'.format(code_site_root_path_url) }}

## Interface Overview

`CoreRef` is the root handle type for invoking framework functionalities, which can be obtained in the following two ways:
- Developers inherit the `ModuleBase` type to implement their own `Module`. In the `Initialize` method, the AimRT framework will pass in a `CoreRef` handle;
- Through the Create Module approach, AimRT will return the corresponding module's `CoreRef` handle after creating a `Module`.

The core interfaces provided by `CoreRef` are as follows:
- `Info()->ModuleInfo`: Get module information;
- `GetConfigurator()->ConfiguratorRef`: Get the configuration handle;
- `GetLogger()->LoggerRef`: Get the logger handle;
- `GetExecutorManager()->ExecutorManagerRef`: Get the executor manager handle;
- `GetRpcHandle()->RpcHandleRef`: Get the RPC handle;
- `GetChannelHandle()->ChannelHandleRef`: Get the Channel handle;

Notes on using `CoreRef`:
- The AimRT framework generates a dedicated `CoreRef` handle for each module to achieve functionalities such as resource isolation and monitoring. The module information it belongs to can be obtained through the `CoreRef::Info` interface.
- The corresponding component handles can be obtained through the `GetXXX` interfaces in `CoreRef` to invoke related functionalities. For specific component documentation, please refer to:
  - [Configurator](./configurator.md)
  - [Executor](./executor.md)
  - [Logger](./logger.md)
  - [Channel](./channel.md)
  - [Rpc](./rpc.md)

## Usage Examples

The following is a simple usage example demonstrating how to obtain and use the `CoreRef` handle when inheriting the `ModuleBase` type:
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

The following is another simple usage example demonstrating how to obtain and use the `CoreRef` handle in App mode:
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