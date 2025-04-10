# CoreRef

## 相关链接

代码文件：
- {{ '[aimrt_module_cpp_interface/core.h]({}/src/interface/aimrt_module_cpp_interface/core.h)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[helloworld_module.cc]({}/src/examples/cpp/helloworld/module/helloworld_module/helloworld_module.cc)'.format(code_site_root_path_url) }}


## 接口概述

`aimrt::CoreRef`是调用框架功能的根句柄类型，可以通过以下两种方式获取：
- 开发者继承`ModuleBase`类型实现自己的`Module`，在`Initialize`方法中，AimRT 框架会传入一个`aimrt::CoreRef`句柄；
- App 模式下，通过 Create Module 方式，AimRT 会在创建一个`Module`后返回对应模块的`aimrt::CoreRef`句柄；

`aimrt::CoreRef`中提供的核心接口如下：

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

`aimrt::CoreRef`的使用注意点：
- AimRT 框架会为每个模块生成一个专属`CoreRef`句柄，以实现资源隔离、监控等方面的功能。可以通过`CoreRef::Info`接口获取其所属的模块的信息。
- 可以通过`CoreRef`中的`GetXXX`接口获取对应组件的句柄，来调用相关功能。具体组件的文档请参考：
  - [Configurator](./configurator.md)
  - [Executor](./executor.md)
  - [Logger](./logger.md)
  - [Parameter](./parameter.md)
  - [Channel](./channel.md)
  - [Rpc](./rpc.md)


## 使用示例

以下是一个简单的使用示例，演示了继承`ModuleBase`类型时如何获取并使用`aimrt::CoreRef`句柄：
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


以下是另一个简单的使用示例，演示了 App 模式下如何获取并使用`aimrt::CoreRef`句柄：
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
