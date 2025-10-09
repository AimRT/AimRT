# Runtime Interface


## Related Links

Code files:
- {{ '[aimrt_core.h]({}/src/runtime/core/aimrt_core.h)'.format(code_site_root_path_url) }}

Reference examples:
- {{ '[helloworld]({}/src/examples/cpp/helloworld)'.format(code_site_root_path_url) }}
  - {{ '[helloworld_app_registration_mode]({}/src/examples/cpp/helloworld/app/helloworld_app_registration_mode)'.format(code_site_root_path_url) }}
  - {{ '[helloworld_app]({}/src/examples/cpp/helloworld/app/helloworld_app)'.format(code_site_root_path_url) }}
  - {{ '[normal_publisher_app]({}/src/examples/cpp/pb_chn/app/normal_publisher_app)'.format(code_site_root_path_url) }}
  - {{ '[normal_subscriber_app]({}/src/examples/cpp/pb_chn/app/normal_subscriber_app)'.format(code_site_root_path_url) }}


## Introduction

If the C++ interface during the logical development phase is mainly for users to develop specific business logic, then the **runtime interface** introduced in this document is for users to decide how to deploy, integrate, and run this business logic.

AimRT provides two deployment and integration methods:
- **App Mode**: The developer registers/creates various modules in their own Main function, and directly compiles the business logic into the main program at compile time;
- **Pkg Mode**: Uses the **aimrt_main** executable provided by AimRT, loads dynamic library form `Pkg` at runtime according to the configuration file, and imports the `Module` inside;


For the advantages, disadvantages, and applicable scenarios of the two methods, please refer to the description in the [Basic Concepts in AimRT](../concepts/concepts.md) document.

Regardless of which method is adopted, it does not affect the business logic, and the two methods can coexist and can be switched relatively easily. The actual method to be adopted needs to be judged according to the specific scenario.

For information about the `aimrt::CoreRef` handle, please refer to the [CoreRef](./core_ref.md) document.

## App Mode

The developer directly references the CMake Target: **aimrt::runtime::core**, and can then use the `aimrt::runtime::core::AimRTCore` class in the {{ '[core/aimrt_core.h]({}/src/runtime/core/aimrt_core.h)'.format(code_site_root_path_url) }} file. The core interfaces needed in App mode are as follows:


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
  - Receives an `AimRTCore::Options` as initialization parameters. The most important item is `cfg_file_path`, used to set the configuration file path.
  - If initialization fails, an exception will be thrown.
- `void Start()`: Starts the framework.
  - If startup fails, an exception will be thrown.
  - Must be called after the Initialize method, otherwise behavior is undefined.
  - If startup is successful, it will block the current thread and use the current thread as the main thread for this `AimRTCore` instance.
- `std::future<void> AsyncStart()`: Starts the framework asynchronously.
  - If startup fails, an exception will be thrown.
  - Must be called after the Initialize method, otherwise behavior is undefined.
  - If startup is successful, it will return a `std::future<void>` handle. After calling the `Shutdown` method, you need to call the `wait` method of this handle to block and wait for the end.
  - This method will internally start a new thread as the main thread for this `AimRTCore` instance.
- `void Shutdown()`: Stops the framework.
  - This method can be called in any thread and at any stage, and can be called any number of times.
  - After calling this method, the `Start` method will exit blocking after completing all tasks in the main thread.
  - Note that sometimes business logic may block tasks in the main thread, causing the `Start` method to be unable to exit blocking and gracefully end the entire framework. In this case, external force kill is needed.

Developers can create an `AimRTCore` instance in their own Main function, call its `Initialize`, `Start`/`AsyncStart` methods in sequence, and can capture the `Ctrl-C` signal themselves to call the `Shutdown` method to achieve graceful exit of the `AimRTCore` instance.

The `GetModuleManager` method of the `AimRTCore` type can return a `ModuleManager` handle, which can be used to register or create modules. In App mode, you need to use the `RegisterModule` interface or `CreateModule` interface it provides:

```cpp
namespace aimrt::runtime::core::module {

class ModuleManager {
 public:
  void RegisterModule(const aimrt_module_base_t* module);

  const aimrt_core_base_t* CreateModule(std::string_view module_name);
};

}  // namespace aimrt::runtime::core::module
```


`RegisterModule` and `CreateModule` represent two ways of writing logic in App mode: **Register Module** mode and **Create Module** mode. The former still requires writing a business module class that inherits from the `ModuleBase` class, while the latter is more flexible.### Register Module

You can register a standard module directly via `RegisterModule`. Developers first need to implement a `Module` that inherits from the `ModuleBase` base class, then register this `Module` instance before the `Initialize` method of the `AimRTCore` instance is called. In this mode, the `Module` boundary remains fairly clear.

For details about the `ModuleBase` base class, please refer to the [ModuleBase](./module_base.md) document.

Below is a simple example; the `main.cc` file that developers need to write is as follows:

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


Compile the above `main.cc`, then launch the resulting executable to run the process; press `ctrl-c` to stop it.

For the complete example code, please see:
- {{ '[helloworld_app_registration_mode]({}/src/examples/cpp/helloworld/app/helloworld_app_registration_mode)'.format(code_site_root_path_url) }}


### Create Module

After the `Initialize` method of the `AimRTCore` instance has been called, you can create a module via `CreateModule`, which returns an `aimrt::CoreRef` handle. Developers can directly invoke certain framework methods—such as RPC or logging—based on this handle. In this mode there is no clear `Module` boundary, which makes it unsuitable for organizing large projects; it is generally used only for quickly building small utilities.

Below is a simple example that implements a channel message publisher; the `main.cc` file that developers need to write is as follows:

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


Compile the above `main.cc`, then launch the resulting executable to run the process. The process will publish one message, wait for a short period, and then exit.

For more examples, please see:
- {{ '[helloworld_app]({}/src/examples/cpp/helloworld/app/helloworld_app)'.format(code_site_root_path_url) }}
- {{ '[normal_publisher_app]({}/src/examples/cpp/pb_chn/app/normal_publisher_app)'.format(code_site_root_path_url) }}
- {{ '[normal_subscriber_app]({}/src/examples/cpp/pb_chn/app/normal_subscriber_app)'.format(code_site_root_path_url) }}


## Pkg Mode### Creating a Pkg

Developers can reference the CMake Target **aimrt::interface::aimrt_pkg_c_interface**. In its header file {{ '[aimrt_pkg_c_interface/pkg_main.h]({}/src/interface/aimrt_pkg_c_interface/pkg_main.h)'.format(code_site_root_path_url) }}, several interfaces that need to be implemented are defined:


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


Among them, `aimrt_module_base_t` can be obtained from a derived class that inherits the `ModuleBase` base class. For information about the `ModuleBase` base class, please refer to the [ModuleBase](./module_base.md) documentation.

Through these interfaces, the AimRT framework runtime can obtain the desired modules from the Pkg dynamic library. Developers need to implement these interfaces in a `C/CPP` file to create a Pkg.

These interfaces are in pure C form, so theoretically, as long as developers hide all symbols of the Pkg, good compatibility between different Pkgs can be achieved. If developers use C++, they can also use a simple macro in the {{ '[aimrt_pkg_c_interface/pkg_macro.h]({}/src/interface/aimrt_pkg_c_interface/pkg_macro.h)'.format(code_site_root_path_url) }} file to encapsulate these details, and users only need to implement a static array containing all module construction methods.

Here is a simple example. Developers need to write the following `pkg_main.cc` file:


```cpp
#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "bar_module.h"
#include "foo_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"FooModule", []() -> aimrt::ModuleBase* { return new FooModule(); }},
    {"BarModule", []() -> aimrt::ModuleBase* { return new BarModule(); }}};

AIMRT_PKG_MAIN(aimrt_module_register_array)
```


### Starting a Pkg

After compiling the above example `pkg_main.cc` into a dynamic library, you can use the **aimrt_main** executable provided by AimRT to start the process, loading the Pkg dynamic library through the path specified in the configuration. Example configuration is as follows:

```yaml
aimrt:
  module:
    pkgs:
      - path: /path/to/your/pkg/libxxx_pkg.so
```


With the configuration file, start the AimRT process using the following example command. Press `ctrl-c` to stop the process:

```shell
./aimrt_main --cfg_file_path=/path/to/your/cfg/xxx_cfg.yaml
```



AimRT officially provides the **aimrt_main** executable program, which accepts some parameters as initialization parameters for the AimRT runtime. The functions of these parameters are as follows:

|  Parameter            |  Type     | Default Value       | Function | Example |
|  ----                 | ----      | ----                | ----     | ----    |
| cfg_file_path         | string    | ""                  | Configuration file path. | --cfg_file_path=/path/to/your/xxx_cfg.yaml |
| dump_cfg_file         | bool      | false               | Whether to dump the configuration file. | --dump_cfg_file=true |
| dump_cfg_file_path    | string    | "./dump_cfg.yaml"   | Path to dump the configuration file. | --dump_cfg_file_path=/path/to/your/xxx_dump_cfg.yaml |
| dump_init_report      | bool      | false               | Whether to dump the initialization report.<br>Note that an initialization report is only available after successful initialization. | --dump_init_report=true |
| dump_init_report_path | string    | "./init_report.txt" | Path to dump the initialization report. | --dump_init_report_path=/path/to/your/xxx_init_report.txt |
| register_signal       | bool      | true                | Whether to register sigint and sigterm signals for triggering shutdown. | --register_signal=true |
| running_duration      | int32     | 0                   | Duration of this run in seconds. If 0, it runs indefinitely. | --running_duration=10 |

Developers can also use the `./aimrt_main --help` command to view the functions of these parameters.