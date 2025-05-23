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

If the C++ interfaces during the logic development phase mainly enable users to develop specific business logic, then the **runtime interfaces** introduced in this document allow users to determine how to deploy, integrate, and run this business logic.

AimRT provides two deployment and integration methods:
- **App Mode**: Developers register/create modules in their own Main function, compiling the business logic directly into the main program during compilation;
- **Pkg Mode**: Uses the **aimrt_main** executable provided by AimRT to load dynamically linked `Pkg` libraries at runtime based on configuration files, importing the `Module` within them.

For the advantages, disadvantages, and applicable scenarios of both methods, please refer to the explanations in the [Basic Concepts in AimRT](../concepts/concepts.md) document.

Neither method affects the business logic, and both can coexist or be switched between relatively easily. The actual choice depends on specific scenarios.

For information about the `aimrt::CoreRef` handle, please refer to the [CoreRef](./core_ref.md) document.

## App Mode

Developers directly reference the CMake Target: **aimrt::runtime::core**, and then can use the `aimrt::runtime::core::AimRTCore` class from the {{ '[core/aimrt_core.h]({}/src/runtime/core/aimrt_core.h)'.format(code_site_root_path_url) }} file. The core interfaces required for App Mode are as follows:

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
  - Accepts an `AimRTCore::Options` as initialization parameters. The most important item is `cfg_file_path`, which sets the configuration file path.
  - Throws an exception if initialization fails.
- `void Start()`: Starts the framework.
  - Throws an exception if startup fails.
  - Must be called after the Initialize method; otherwise, behavior is undefined.
  - If successful, it blocks the current thread and uses it as the main thread for this `AimRTCore` instance.
- `std::future<void> AsyncStart()`: Asynchronously starts the framework.
  - Throws an exception if startup fails.
  - Must be called after the Initialize method; otherwise, behavior is undefined.
  - If successful, returns a `std::future<void>` handle. The `wait` method of this handle must be called to block and wait for completion after calling the `Shutdown` method.
  - This method internally starts a new thread as the main thread for this `AimRTCore` instance.
- `void Shutdown()`: Stops the framework.
  - Can be called from any thread at any stage and can be called any number of times.
  - After calling this method, the `Start` method will exit the blocking state after completing all tasks in the main thread.
  - Note: Sometimes business logic may block tasks in the main thread, preventing the `Start` method from exiting the blocking state and gracefully terminating the framework. In such cases, an external force kill may be required.

Developers can create an `AimRTCore` instance in their Main function, sequentially call its `Initialize`, `Start`/`AsyncStart` methods, and capture the `Ctrl-C` signal themselves to call the `Shutdown` method, enabling graceful exit of the `AimRTCore` instance.

The `GetModuleManager` method of the `AimRTCore` type returns a `ModuleManager` handle, which can be used to register or create modules. In App Mode, the `RegisterModule` or `CreateModule` interfaces are required:

```cpp
namespace aimrt::runtime::core::module {

class ModuleManager {
 public:
  void RegisterModule(const aimrt_module_base_t* module);

  const aimrt_core_base_t* CreateModule(std::string_view module_name);
};

}  // namespace aimrt::runtime::core::module
```

`RegisterModule` and `CreateModule` represent two ways of writing logic in App Mode: **Module Registration** and **Module Creation**. The former still requires writing a business module class that inherits from `ModuleBase`, while the latter offers more freedom.### Registration Module

Through `RegisterModule`, you can directly register a standard module. Developers need to first implement a `Module` that inherits from the base class `ModuleBase`, then register this `Module` instance before the `AimRTCore` instance calls the `Initialize` method. This approach still maintains a relatively clear `Module` boundary.

For information about the `ModuleBase` base class, please refer to the [ModuleBase](./module_base.md) documentation.

Here is a simple example. The `main.cc` file that developers need to write is as follows:
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

Compile the above `main.cc` example, and simply run the compiled executable to start the process. Press `ctrl-c` to stop the process.

For detailed example code, please refer to:
- {{ '[helloworld_app_registration_mode]({}/src/examples/cpp/helloworld/app/helloworld_app_registration_mode)'.format(code_site_root_path_url) }}

### Creating a Module

After the `AimRTCore` instance calls the `Initialize` method, you can create a module using `CreateModule`, which returns an `aimrt::CoreRef` handle. Developers can directly use this handle to call some framework methods, such as RPC or Log. This approach does not have a clear `Module` boundary, which is not conducive to organizing large projects. It is generally only used for quickly creating small tools.

Here is a simple example that implements a function to publish channel messages. The `main.cc` file that developers need to write is as follows:
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

Compile the above `main.cc` example, and simply run the compiled executable to start the process. The process will publish a message, wait for a period of time, and then exit.

For more examples, please refer to:
- {{ '[helloworld_app]({}/src/examples/cpp/helloworld/app/helloworld_app)'.format(code_site_root_path_url) }}
- {{ '[normal_publisher_app]({}/src/examples/cpp/pb_chn/app/normal_publisher_app)'.format(code_site_root_path_url) }}
- {{ '[normal_subscriber_app]({}/src/examples/cpp/pb_chn/app/normal_subscriber_app)'.format(code_site_root_path_url) }}

## Pkg Mode### Creating a Pkg

Developers can reference the CMake Target: **aimrt::interface::aimrt_pkg_c_interface**. In its header file {{ '[aimrt_pkg_c_interface/pkg_main.h]({}/src/interface/aimrt_pkg_c_interface/pkg_main.h)'.format(code_site_root_path_url) }}, several interfaces to be implemented are defined:

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

Here, `aimrt_module_base_t` can be obtained from a derived class that inherits from the `ModuleBase` base class. For information about the `ModuleBase` base class, please refer to the [ModuleBase](./module_base.md) documentation.

Through these interfaces, the AimRT framework can retrieve the desired modules from the Pkg dynamic library at runtime. Developers need to implement these interfaces in a `C/CPP` file to create a Pkg.

These interfaces are in pure C form, so theoretically, as long as developers hide all symbols of the Pkg, good compatibility between different Pkgs can be achieved. If developers use C++, they can also use a simple macro from the {{ '[aimrt_pkg_c_interface/pkg_macro.h]({}/src/interface/aimrt_pkg_c_interface/pkg_macro.h)'.format(code_site_root_path_url) }} file to encapsulate these details. Users only need to implement a static array containing all module constructors.

Below is a simple example. Developers need to write a `pkg_main.cc` file as follows:

```cpp
#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "bar_module.h"
#include "foo_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"FooModule", []() -> aimrt::ModuleBase* { return new FooModule(); }},
    {"BarModule", []() -> aimrt::ModuleBase* { return new BarModule(); }}};

AIMRT_PKG_MAIN(aimrt_module_register_array)
```

### Launching a Pkg

After compiling the above example `pkg_main.cc` into a dynamic library, you can use the **aimrt_main** executable provided by AimRT to start the process, loading the Pkg dynamic library via the path specified in the configuration. An example configuration is as follows:
```yaml
aimrt:
  module:
    pkgs:
      - path: /path/to/your/pkg/libxxx_pkg.so
```

With the configuration file in place, you can start the AimRT process using the following example command. Press `ctrl-c` to stop the process:
```shell
./aimrt_main --cfg_file_path=/path/to/your/cfg/xxx_cfg.yaml
```

The **aimrt_main** executable provided by AimRT accepts several parameters as initialization parameters for the AimRT runtime. The functions of these parameters are as follows:

|  Parameter            |  Type     | Default             | Purpose  | Example |
|  ----                 | ----      | ----                | ----  | ----  |
| cfg_file_path         | string    | ""                  | Path to the configuration file.  |  --cfg_file_path=/path/to/your/xxx_cfg.yaml |
| dump_cfg_file         | bool      | false               | Whether to dump the configuration file. |  --dump_cfg_file=true |
| dump_cfg_file_path    | string    | "./dump_cfg.yaml"   | Path to dump the configuration file. |  --dump_cfg_file_path=/path/to/your/xxx_dump_cfg.yaml |
| dump_init_report      | bool      | false               | Whether to dump the initialization report.<br>Note: The initialization report is only available if initialization succeeds. |  --dump_init_report=true |
| dump_init_report_path | string    | "./init_report.txt" | Path to dump the initialization report. |  --dump_init_report_path=/path/to/your/xxx_init_report.txt |
| register_signal       | bool      | true                | Whether to register sigint and sigterm signals to trigger Shutdown. |  --register_signal=true |
| running_duration      | int32     | 0                   | Duration of this run, in seconds. If 0, it runs indefinitely. |  --running_duration=10 |

Developers can also use the `./aimrt_main --help` command to view the functions of these parameters.