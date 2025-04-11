

# Runtime Interface

## Related Links

Code Files:
- {{ '[aimrt_core.h]({}/src/runtime/core/aimrt_core.h)'.format(code_site_root_path_url) }}

Reference Examples:
- {{ '[helloworld]({}/src/examples/cpp/helloworld)'.format(code_site_root_path_url) }}
  - {{ '[helloworld_app_registration_mode]({}/src/examples/cpp/helloworld/app/helloworld_app_registration_mode)'.format(code_site_root_path_url) }}
  - {{ '[helloworld_app]({}/src/examples/cpp/helloworld/app/helloworld_app)'.format(code_site_root_path_url) }}
  - {{ '[normal_publisher_app]({}/src/examples/cpp/pb_chn/app/normal_publisher_app)'.format(code_site_root_path_url) }}
  - {{ '[normal_subscriber_app]({}/src/examples/cpp/pb_chn/app/normal_subscriber_app)'.format(code_site_root_path_url) }}

## Introduction

If the C++ interfaces during logical development phase mainly enable users to develop specific business logic, the **runtime interfaces** described in this document allow users to decide how to deploy, integrate, and execute these business logics.

AimRT provides two deployment integration modes:
- **App Mode**: Developers register/create modules in their own main function, compiling business logic directly into the main program during compilation;
- **Pkg Mode**: Uses the **aimrt_main** executable provided by AimRT to load dynamically linked `Pkg` packages containing `Module` implementations at runtime based on configuration files;

For advantages/disadvantages and applicable scenarios of both modes, please refer to the documentation in [Basic Concepts in AimRT](../concepts/concepts.md).

Neither approach affects business logic, and both modes can coexist or switch between each other easily. The actual choice should be determined based on specific scenarios.

Regarding the `aimrt::CoreRef` handle, please refer to the [CoreRef](./core_ref.md) documentation.

## App Mode

Developers directly reference the CMake Target: **aimrt::runtime::core**, then use the `aimrt::runtime::core::AimRTCore` class from {{ '[core/aimrt_core.h]({}/src/runtime/core/aimrt_core.h)'.format(code_site_root_path_url) }}. Core interfaces required in App Mode are as follows:

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

Interface usage instructions:
- `void Initialize(const Options& options)`: Used to initialize the framework.
  - Accepts an `AimRTCore::Options` as initialization parameter. The most important item is `cfg_file_path` for setting configuration file path.
  - Throws an exception if initialization fails.
- `void Start()`: Starts the framework.
  - Throws an exception if startup fails.
  - Must be called after Initialize method, otherwise behavior is undefined.
  - Blocks current thread upon successful startup, using it as the main thread for this `AimRTCore` instance.
- `std::future<void> AsyncStart()`: Asynchronously starts the framework.
  - Throws an exception if startup fails.
  - Must be called after Initialize method, otherwise behavior is undefined.
  - Returns a `std::future<void>` handle upon success. Call `wait` on this handle after `Shutdown` to block until completion.
  - Starts a new internal thread as the main thread for this `AimRTCore` instance.
- `void Shutdown()`: Stops the framework.
  - Can be called from any thread at any stage, any number of times.
  - After calling this method, the `Start` method will unblock after completing all tasks in the main thread.
  - Note: Business logic might block main thread tasks, preventing `Start` from unblocking. In such cases, external kill may be required.

Developers can create an `AimRTCore` instance in their main function, sequentially call `Initialize`, `Start`/`AsyncStart` methods, and handle `Ctrl-C` signals to call `Shutdown` for graceful exit.

The `GetModuleManager` method of `AimRTCore` returns a `ModuleManager` handle for module registration/creation. App Mode requires using its `RegisterModule` or `CreateModule` interfaces:
```cpp
namespace aimrt::runtime::core::module {

class ModuleManager {
 public:
  void RegisterModule(const aimrt_module_base_t* module);

  const aimrt_core_base_t* CreateModule(std::string_view module_name);
};

}  // namespace aimrt::runtime::core::module
```

`RegisterModule` and `CreateModule` represent two approaches in App Mode: **Registration Mode** requires writing business module classes inheriting from `ModuleBase`, while **Creation Mode** offers more flexibility.

### Registration Module

The `RegisterModule` method allows direct registration of a standard module. Developers need to first implement a `Module` that inherits from the `ModuleBase` base class, then register this module instance before calling the `Initialize` method of the `AimRTCore` instance. This approach maintains clear module boundaries.

For information about the `ModuleBase` base class, please refer to the [ModuleBase](./module_base.md) documentation.

Here's a simple example. Developers need to create a `main.cc` file as follows:
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

After compiling the above `main.cc` example, simply execute the compiled binary to start the process. Press `ctrl-c` to terminate the process.

Detailed example code can be found at:
- {{ '[helloworld_app_registration_mode]({}/src/examples/cpp/helloworld/app/helloworld_app_registration_mode)'.format(code_site_root_path_url) }}

### Creating Modules

After the `AimRTCore` instance calls the `Initialize` method, developers can use `CreateModule` to create a module and obtain an `aimrt::CoreRef` handle. This handle can be used to call various framework methods like RPC or Log. This approach lacks clear module boundaries and is not ideal for large projects, but suitable for quickly building small tools.

Here's a simple example implementing a channel message publisher. Developers need to create a `main.cc` file as follows:
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

After compiling the above `main.cc` example, executing the compiled binary will start a process that publishes a message, waits for some time, then exits.

More examples can be found at:
- {{ '[helloworld_app]({}/src/examples/cpp/helloworld/app/helloworld_app)'.format(code_site_root_path_url) }}
- {{ '[normal_publisher_app]({}/src/examples/cpp/pb_chn/app/normal_publisher_app)'.format(code_site_root_path_url) }}
- {{ '[normal_subscriber_app]({}/src/examples/cpp/pb_chn/app/normal_subscriber_app)'.format(code_site_root_path_url) }}

## Pkg Mode

### Creating a Pkg

Developers can reference the CMake Target: **aimrt::interface::aimrt_pkg_c_interface**. The header file {{ '[aimrt_pkg_c_interface/pkg_main.h]({}/src/interface/aimrt_pkg_c_interface/pkg_main.h)'.format(code_site_root_path_url) }} defines several interfaces to implement:

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

The `aimrt_module_base_t` can be obtained from derived classes that inherit the `ModuleBase` base class. For information about the `ModuleBase` base class, please refer to the [ModuleBase](./module_base.md) documentation.


Through these interfaces, the AimRT framework runtime can obtain required modules from the Pkg dynamic library. Developers need to implement these interfaces in a `C/CPP` file to create a Pkg.


These interfaces use pure C format, so theoretically different Pkgs can maintain good compatibility as long as developers hide all Pkg symbols. For C++ developers, they can use macros from {{ '[aimrt_pkg_c_interface/pkg_macro.h]({}/src/interface/aimrt_pkg_c_interface/pkg_macro.h)'.format(code_site_root_path_url) }} to encapsulate implementation details - simply implement a static array containing all module construction methods.


Here is a simple example. Developers need to create a `pkg_main.cc` file like this:

```cpp
#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "bar_module.h"
#include "foo_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"FooModule", []() -> aimrt::ModuleBase* { return new FooModule(); }},
    {"BarModule", []() -> aimrt::ModuleBase* { return new BarModule(); }}};

AIMRT_PKG_MAIN(aimrt_module_register_array)
```

### Launching Pkg

After compiling the example `pkg_main.cc` into a dynamic library, use the **aimrt_main** executable provided by AimRT to launch the process, loading the Pkg dynamic library through the path specified in the configuration. Example configuration:

```yaml
aimrt:
  module:
    pkgs:
      - path: /path/to/your/pkg/libxxx_pkg.so
```

With the configuration file prepared, use the following command to start the AimRT process (press `ctrl-c` to stop):

```shell
./aimrt_main --cfg_file_path=/path/to/your/cfg/xxx_cfg.yaml
```


The official **aimrt_main** executable accepts several parameters for AimRT runtime initialization:

|  Parameter            |  Type     | Default             | Function  | Example |
|  ----                 | ----      | ----                | ----      | ----    |
| cfg_file_path         | string    | ""                  | Configuration file path | --cfg_file_path=/path/to/your/xxx_cfg.yaml |
| dump_cfg_file         | bool      | false               | Whether to dump configuration files | --dump_cfg_file=true |
| dump_cfg_file_path    | string    | "./dump_cfg.yaml"   | Dump configuration file path | --dump_cfg_file_path=/path/to/your/xxx_dump_cfg.yaml |
| dump_init_report      | bool      | false               | Whether to dump initialization report<br>Note: Only available after successful initialization | --dump_init_report=true |
| dump_init_report_path | string    | "./init_report.txt" | Initialization report dump path | --dump_init_report_path=/path/to/your/xxx_init_report.txt |
| register_signal       | bool      | true                | Whether to register sigint and sigterm signals for triggering shutdown | --register_signal=true |
| running_duration      | int32     | 0                   | Runtime duration in seconds (0 means run indefinitely) | --running_duration=10 |


Developers can also view parameter descriptions using `./aimrt_main --help`.