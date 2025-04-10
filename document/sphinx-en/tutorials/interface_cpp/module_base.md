# ModuleBase


## 相关链接

代码文件：
- {{ '[aimrt_module_cpp_interface/module_base.h]({}/src/interface/aimrt_module_cpp_interface/module_base.h)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[helloworld_module.cc]({}/src/examples/cpp/helloworld/module/helloworld_module/helloworld_module.cc)'.format(code_site_root_path_url) }}


## 接口概述

`ModuleBase`类型是一个模块基类类型，开发者可以继承`ModuleBase`类型来实现自己的`Module`，它定义了业务模块所需要实现的几个接口：

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

其中`aimrt::ModuleInfo`结构体声明如下：
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

关于这些虚接口，说明如下：
- `ModuleInfo Info()`：用于 AimRT 获取模块信息，包括模块名称、模块版本等；
  - AimRT 会在加载模块时调用 `Info` 接口，读取模块信息；
  - `ModuleInfo`结构中除`name`是必须项，其余都是可选项；
  - 如果模块在其中抛了异常，等效于返回一个空 ModuleInfo；
- `bool Initialize(CoreRef)`：用于初始化模块；
  - AimRT 在 Initialize 阶段，依次调用各模块的 `Initialize` 方法；
  - AimRT 会在主线程中调用模块的 `Initialize` 方法，模块不应阻塞 `Initialize` 方法太久；
  - AimRT 在调用模块 `Initialize` 方法时，会传入一个 CoreRef 句柄，模块可以存储此句柄，并在后续通过它调用框架的功能；
  - 在 AimRT 调用模块的 `Initialize` 方法之前，所有的组件（例如配置、日志等）都已经完成 Initialize，但还未 Start；
  - 如果模块在 `Initialize` 方法中抛了异常，等效于返回 false；
  - 如果有任何模块在 AimRT 调用其 `Initialize` 方法时返回了 false，则整个 AimRT 会 Initialize 失败并退出；
- `bool Start()`：用于启动模块；
  - AimRT 在 Start 阶段依次调用各模块的 `Start` 方法；
  - AimRT 会在主线程中调用模块的 `Start` 方法，模块不应阻塞 `Start` 方法太久；
  - 在 AimRT 调用模块的 `Start` 方法之前，所有的组件（例如配置、日志等）都已经进入 Start 阶段；
  - 如果模块在 `Start` 方法中抛了异常，等效于返回了 false；
  - 如果有任何模块在 AimRT 调用其 `Start` 方法时返回了 false，则整个 AimRT 会 Start 失败并退出；
- `void Shutdown()`：用于停止模块，一般用于整个进程的优雅退出；
  - AimRT 在 Shutdown 阶段依次调用各个模块的 `Shutdown` 方法；
  - AimRT 会在主线程中调用模块的 `Shutdown` 方法，模块不应阻塞 `Shutdown` 方法太久；
  - AimRT 可能在任何阶段直接进入 Shutdown 阶段；
  - 如果模块在 `Shutdown` 方法中抛了异常，AimRT 会 catch 住并直接返回；
  - 在 AimRT 调用模块的 `Shutdown` 方法之后，各个组件（例如配置、日志等）才会 Shutdown；


## 使用示例

以下是一个简单的示例，实现了一个最基础的 HelloWorld 模块：
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

