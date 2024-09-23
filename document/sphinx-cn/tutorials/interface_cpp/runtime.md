
# Runtime Interface


## 相关链接

代码文件：
- {{ '[aimrt_core.h]({}/src/runtime/core/aimrt_core.h)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[helloworld]({}/src/examples/cpp/helloworld)'.format(code_site_root_path_url) }}
  - {{ '[helloworld_app_registration_mode]({}/src/examples/cpp/helloworld/app/helloworld_app_registration_mode)'.format(code_site_root_path_url) }}
  - {{ '[helloworld_app]({}/src/examples/cpp/helloworld/app/helloworld_app)'.format(code_site_root_path_url) }}
  - {{ '[normal_publisher_app]({}/src/examples/cpp/protobuf_channel/app/normal_publisher_app)'.format(code_site_root_path_url) }}
  - {{ '[normal_subscriber_app]({}/src/examples/cpp/protobuf_channel/app/normal_subscriber_app)'.format(code_site_root_path_url) }}


## 简介

如果说逻辑开发阶段的 C++ 接口主要是让用户开发具体的业务逻辑，那么本文档所介绍的**运行时接口**则是让用户决定如何部署、集成、运行这些业务逻辑。

AimRT 提供了两种部署集成方式：
- **App 模式**：开发者在自己的 Main 函数中注册/创建各个模块，编译时直接将业务逻辑编译进主程序；
- **Pkg 模式**：使用 AimRT 提供的**aimrt_main**可执行程序，在运行时根据配置文件加载动态库形式的`Pkg`，导入其中的`Module`；


两者的优劣势和适用场景请参考[AimRT 中的基本概念](../concepts/concepts.md)文档里的说明。


无论采用哪种方式都不影响业务逻辑，且两种方式可以共存，也可以比较简单的进行切换，实际采用哪种方式需要根据具体场景进行判断。


关于`aimrt::CoreRef`句柄的相关信息，请参考[CoreRef](./core_ref.md)文档。

## App 模式

开发者直接引用 CMake Target：**aimrt::runtime::core**，然后即可使用{{ '[core/aimrt_core.h]({}/src/runtime/core/aimrt_core.h)'.format(code_site_root_path_url) }}文件中的`aimrt::runtime::core::AimRTCore`类，在 App 模式下需要使用的核心接口如下：

```cpp
namespace aimrt::runtime::core {

class AimRTCore {
 public:
  struct Options {
    std::string cfg_file_path;
  };

 public:
  void Initialize(const Options& options);

  void Start();
  std::future<void> AsyncStart();

  void Shutdown();

  // ...

  module::ModuleManager& GetModuleManager();

  // ...
};

}  // namespace aimrt::runtime::core
```

接口使用说明如下：
- `void Initialize(const Options& options)`：用于初始化框架。
  - 接收一个`AimRTCore::Options`作为初始化参数。其中最重要的项是`cfg_file_path`，用于设置配置文件路径。
  - 如果初始化失败，会抛出一个异常。
- `void Start()`：启动框架。
  - 如果启动失败，会抛出一个异常。
  - 必须在 Initialize 方法之后调用，否则行为未定义。
  - 如果启动成功，会阻塞当前线程，并将当前线程作为本`AimRTCore`实例的主线程。
- `std::future<void> AsyncStart()`：异步启动框架。
  - 如果启动失败，会抛出一个异常。
  - 必须在 Initialize 方法之后调用，否则行为未定义。
  - 如果启动成功，会返回一个`std::future<void>`句柄，在外部可以调用该句柄的`wait`方法阻塞等待结束。
  - 该方法会在内部新启动一个线程作为本`AimRTCore`实例的主线程。
- `void Shutdown()`：停止框架。
  - 可以在任意线程、任意阶段调用此方法，也可以调用任意次数。
  - 调用此方法后，`Start`方法将在执行完主线程中的所有任务后，退出阻塞。
  - 需要注意，有时候业务会阻塞住主线程中的任务，导致`Start`方法无法退出阻塞、优雅结束整个框架，此时需要在外部强制 kill。


开发者可以在自己的 Main 函数中创建一个`AimRTCore`实例，依次调用其`Initialize`、`Start`/`AsyncStart`方法，并可以自己捕获`Ctrl-C`信号来调用`Shutdown`方法，以实现`AimRTCore`实例的优雅退出。

`AimRTCore`类型的`GetModuleManager`方法可以返回一个`ModuleManager`句柄，可以用来注册或创建模块，App 模式下需要使用其提供的`RegisterModule`接口或`CreateModule`接口：
```cpp
namespace aimrt::runtime::core::module {

class ModuleManager {
 public:
  void RegisterModule(const aimrt_module_base_t* module);

  const aimrt_core_base_t* CreateModule(std::string_view module_name);
};

}  // namespace aimrt::runtime::core::module
```

`RegisterModule`和`CreateModule`代表了 App 模式下编写逻辑的两种方式：**注册模块**方式与**创建模块**方式，前者仍然需要编写一个继承于`ModuleBase`类的业务模块类，后者则更加自由。

### 注册模块

通过`RegisterModule`可以直接注册一个标准模块。开发者需要先实现一个继承于`ModuleBase`基类的`Module`，然后在`AimRTCore`实例调用`Initialize`方法之前注册该`Module`实例，在此方式下仍然有一个比较清晰的`Module`边界。


关于`ModuleBase`基类的相关信息，请参考[ModuleBase](./module_base.md)文档。


以下是一个简单的例子，开发者需要编写的`main.cc`文件如下：
```cpp
#include <csignal>

#include "core/aimrt_core.h"
#include "aimrt_module_cpp_interface/module_base.h"

AimRTCore* global_core_ptr_ = nullptr;

void SignalHandler(int sig) {
  if (global_core_ptr_ && (sig == SIGINT || sig == SIGTERM)) {
    global_core_ptr_->Shutdown();
    return;
  }
  raise(sig);
};

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

int32_t main(int32_t argc, char** argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  // Create AimRTCore ins
  AimRTCore core;
  global_core_ptr_ = &core;

  // Register module
  HelloWorldModule helloworld_module;
  core.GetModuleManager().RegisterModule(helloworld_module.NativeHandle());

  // Initialize AimRTCore ins
  AimRTCore::Options options;
  options.cfg_file_path = "path/to/cfg/xxx_cfg.yaml";
  core.Initialize(options);

  // Start AimRTCore ins, will block here
  core.Start();

  // Shutdown AimRTCore ins
  core.Shutdown();

  return 0;
}
```

编译上面示例的`main.cc`，直接启动编译后的可执行文件即可运行进程，按下`ctrl-c`后即可停止进程。


详细示例代码请参考：
- {{ '[helloworld_app_registration_mode]({}/src/examples/cpp/helloworld/app/helloworld_app_registration_mode)'.format(code_site_root_path_url) }}


### 创建模块

在`AimRTCore`实例调用`Initialize`方法之后，通过`CreateModule`可以创建一个模块，并返回一个`aimrt::CoreRef`句柄，开发者可以直接基于此句柄调用一些框架的方法，比如 RPC 或者 Log 等。在此方式下没有一个比较清晰的`Module`边界，不利于大型项目的组织，一般仅用于快速做一些小工具。



以下是一个简单的例子，实现了一个发布 channel 消息的功能，开发者需要编写的`main.cc`文件如下：
```cpp
#include "core/aimrt_core.h"

#include "aimrt_module_cpp_interface/core.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"

#include "event.pb.h"

int32_t main(int32_t argc, char** argv) {
  // Create AimRTCore ins
  AimRTCore core;

  // Initialize AimRTCore ins
  AimRTCore::Options options;
  options.cfg_file_path = "path/to/cfg/xxx_cfg.yaml";
  core.Initialize(options);

  // Create module
  aimrt::CoreRef core_handle(core.GetModuleManager().CreateModule("HelloWorldModule"));

  // Register a msg type for publish
  auto publisher = core_handle.GetChannelHandle().GetPublisher("test_topic");
  aimrt::channel::RegisterPublishType<ExampleEventMsg>(publisher);

  // Start AimRTCore ins
  auto fu = core.AsyncStart();

  // Publish a message
  ExampleEventMsg msg;
  msg.set_msg("example msg");
  aimrt::channel::Publish(publisher, msg);

  // Wait for seconds
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // Shutdown AimRTCore ins
  core.Shutdown();

  // Wait for complete shutdown
  fu.wait();

  return 0;
}
```

编译上面示例的`main.cc`，直接启动编译后的可执行文件即可运行进程，该进程将在发布一个消息后，等待一段时间并退出。

更多示例请参考：
- {{ '[helloworld_app]({}/src/examples/cpp/helloworld/app/helloworld_app)'.format(code_site_root_path_url) }}
- {{ '[normal_publisher_app]({}/src/examples/cpp/protobuf_channel/app/normal_publisher_app)'.format(code_site_root_path_url) }}
- {{ '[normal_subscriber_app]({}/src/examples/cpp/protobuf_channel/app/normal_subscriber_app)'.format(code_site_root_path_url) }}


## Pkg 模式

### 创建 Pkg

开发者可以引用 CMake Target：**aimrt::interface::aimrt_pkg_c_interface**，在其中的头文件{{ '[aimrt_pkg_c_interface/pkg_main.h]({}/src/interface/aimrt_pkg_c_interface/pkg_main.h)'.format(code_site_root_path_url) }}中，定义了几个要实现的接口：

```cpp
#ifdef __cplusplus
extern "C" {
#endif

// Get the num of modules in the pkg
size_t AimRTDynlibGetModuleNum();

// Get the list of module names in the pkg
const aimrt_string_view_t* AimRTDynlibGetModuleNameList();

// Create a module with the name
const aimrt_module_base_t* AimRTDynlibCreateModule(aimrt_string_view_t module_name);

// Destroy module
void AimRTDynlibDestroyModule(const aimrt_module_base_t* module_ptr);

#ifdef __cplusplus
}
#endif
```

其中，`aimrt_module_base_t`可以由继承了`ModuleBase`基类的派生类获得。关于`ModuleBase`基类的相关信息，请参考[ModuleBase](./module_base.md)文档。


通过这些接口，AimRT 框架运行时可以从 Pkg 动态库中获取想要的模块。开发者需要在一个`C/CPP`文件中实现这些接口来创建一个 Pkg。


这些接口是纯 C 形式的，所以理论上只要开发者将 Pkg 的符号都隐藏起来，不同 Pkg 之间可以做到较好的兼容性。如果开发者使用 C++，也可以使用{{ '[aimrt_pkg_c_interface/pkg_macro.h]({}/src/interface/aimrt_pkg_c_interface/pkg_macro.h)'.format(code_site_root_path_url) }}文件中的一个简单的宏来封装这些细节，用户只需要实现一个包含所有模块构造方法的静态数组即可。


以下是一个简单的示例，开发者需要编写如下的`pkg_main.cc`文件：

```cpp
#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "bar_module.h"
#include "foo_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"FooModule", []() -> aimrt::ModuleBase* { return new FooModule(); }},
    {"BarModule", []() -> aimrt::ModuleBase* { return new BarModule(); }}};

AIMRT_PKG_MAIN(aimrt_module_register_array)
```

### 启动 Pkg

将上面的示例`pkg_main.cc`编译为动态库后，即可使用 AimRT 提供的**aimrt_main**可执行程序启动进程，通过配置中指定的路径来加载 Pkg 动态库。示例配置如下：
```yaml
aimrt:
  module:
    pkgs:
      - path: /path/to/your/pkg/libxxx_pkg.so
```

有了配置文件之后，通过以下示例命令启动 AimRT 进程，按下`ctrl-c`后即可停止进程：
```shell
./aimrt_main --cfg_file_path=/path/to/your/cfg/xxx_cfg.yaml
```


AimRT官方提供**aimrt_main**可执行程序接收一些参数作为 AimRT 运行时的初始化参数，这些参数功能如下：

|  参数项               |  类型     | 默认值                |作用  | 示例 |
|  ----                 | ----      | ----                | ----  | ----  |
| cfg_file_path         | string    | ""                  | 配置文件路径。  |  --cfg_file_path=/path/to/your/xxx_cfg.yaml |
| dump_cfg_file         | bool      | false               | 是否 Dump 配置文件。 |  --dump_cfg_file=true |
| dump_cfg_file_path    | string    | "./dump_cfg.yaml"   | Dump 配置文件的路径。 |  --dump_cfg_file_path=/path/to/your/xxx_dump_cfg.yaml |
| dump_init_report      | bool      | false               | 是否 Dump 初始化报告。<br>请注意，仅当初始化成功后才有初始化报告。 |  --dump_init_report=true |
| dump_init_report_path | string    | "./init_report.txt" | Dump 初始化报告的路径。 |  --dump_init_report_path=/path/to/your/xxx_init_report.txt |
| register_signal       | bool      | true                | 是否注册 sigint 和 sigterm 信号，用于触发 Shutdown。 |  --register_signal=true |
| running_duration      | int32     | 0                   | 本次运行时间，单位：s。如果为 0 则表示一直运行。 |  --running_duration=10 |


开发者也可以使用`./aimrt_main --help`命令查看这些参数功能。

