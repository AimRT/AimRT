# CoreRef

## Related Links

Code Files:
- {{ '[aimrt_module_cpp_interface/core.h]({}/src/interface/aimrt_module_cpp_interface/core.h)'.format(code_site_root_path_url) }}

Reference Examples:
- {{ '[helloworld_module.cc]({}/src/examples/cpp/helloworld/module/helloworld_module/helloworld_module.cc)'.format(code_site_root_path_url) }}


## Interface Overview

`aimrt::CoreRef` is the root handle type for invoking framework functionality, which can be obtained in two ways:
- Developers inherit the `ModuleBase` type to implement their own `Module`. In the `Initialize` method, the AimRT framework will pass in an `aimrt::CoreRef` handle;
- In App mode, through the Create Module approach, AimRT will return the corresponding module's `aimrt::CoreRef` handle after creating a `Module`.

The core interfaces provided by `aimrt::CoreRef` are as follows:

```cpp
namespace aimrt {

class CoreRef {
 public:
  ModuleInfo Info() const;

  configurator::ConfiguratorRef GetConfigurator() const;

  executor::ExecutorManagerRef GetExecutorManager() const;

  logger::LoggerRef GetLogger() const;

  rpc::RpcHandleRef GetRpcHandle() const;

  channel::ChannelHandleRef GetChannelHandle() const;

  parameter::ParameterHandleRef GetParameterHandle() const;
};

}  // namespace aimrt
```

Notes on using `aimrt::CoreRef`:
- The AimRT framework generates a dedicated `CoreRef` handle for each module to achieve functionality such as resource isolation and monitoring. The module information it belongs to can be obtained through the `CoreRef::Info` interface.
- The corresponding component handles can be obtained through the `GetXXX` interfaces in `CoreRef` to invoke related functionalities. For specific component documentation, please refer to:
  - [Configurator](./configurator.md)
  - [Executor](./executor.md)
  - [Logger](./logger.md)
  - [Parameter](./parameter.md)
  - [Channel](./channel.md)
  - [Rpc](./rpc.md)


## Usage Examples

The following is a simple usage example demonstrating how to obtain and use the `aimrt::CoreRef` handle when inheriting the `ModuleBase` type:
```cpp
class HelloWorldModule : public aimrt::ModuleBase {
 public:
  // ...

  bool Initialize(aimrt::CoreRef core) override {
    // Get log handle
    auto logger = core.GetLogger();

    // Use log handle
    AIMRT_HL_INFO(logger, "This is a test log");

    return true;
  }
};
```


Here is another simple usage example demonstrating how to obtain and use the `aimrt::CoreRef` handle in App mode:
```cpp
int32_t main(int32_t argc, char** argv) {
  AimRTCore core;

  // Initialize
  AimRTCore::Options options;
  if (argc > 1) options.cfg_file_path = argv[1];
  core.Initialize(options);

  // Get CoreRef handle
  aimrt::CoreRef module_handle(core.GetModuleManager().CreateModule("HelloWorldModule"));

  // Get log handle
  auto logger = module_handle.GetLogger();
  
  // Use log handle
  AIMRT_HL_INFO(logger, "This is a test log");

  // ...
}
```