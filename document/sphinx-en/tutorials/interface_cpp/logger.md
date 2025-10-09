# Logger

## Related Links

Code files:
- {{ '[util/log_util.h]({}/src/common/util/log_util.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/logger/logger.h]({}/src/interface/aimrt_module_cpp_interface/logger/logger.h)'.format(code_site_root_path_url) }}

Reference examples:
- {{ '[helloworld_module.cc]({}/src/examples/cpp/helloworld/module/helloworld_module/helloworld_module.cc)'.format(code_site_root_path_url) }}


## Stand-alone Logger Component in AimRT

AimRT provides a stand-alone, general-purpose logger component that belongs to the **aimrt::common::util** CMake target. Simply `#include "util/log_util.h"` to use it independently of the CPP interface layer.

It offers some basic logging macros that require passing a `Logger` object at the call site to define the concrete behavior of log printing. The logger handle is defined as a template Concept; any C++ class instance that resembles the following example and contains the two interfaces `GetLogLevel` and `Log` can serve as a logger handle:


```cpp
class YourLogger {
 public:
  uint32_t GetLogLevel() const {
    // ...
  }

  void Log(uint32_t lvl, uint32_t line,
           const char* file_name, const char* function_name,
           const char* log_data, size_t log_data_size) const {
    // ...
  }

};
```


Log levels are divided into six tiers:
- Trace
- Debug
- Info
- Warn
- Error
- Fatal

Once you have a logger handle, you can either print logs directly via the `Log` method provided by the handle, or use the supplied logging macros for more convenient printing. Note that these macros rely on C++20 Format syntax; for detailed usage of C++20 Format syntax, please refer to the [C++ official documentation](https://en.cppreference.com/w/cpp/utility/format).

AimRT also provides two default `Logger` types in the `util/log_util.h` file:
- **SimpleLogger**: a simple synchronous logger handle;
- **SimpleAsyncLogger**: a simple asynchronous logger handle;

These two logger handles are typically used in scenarios such as unit tests when the AimRT instance is not yet started.


## Usage Examples of the Stand-alone Logger Component in AimRT

Below are some usage examples:

```cpp
#include "util/log_util.h"

int Main() {
  // Use a simple log handle provided in 'util/log_util.h', which will synchronously print logs on the console
  auto lgr = aimrt::common::util::SimpleLogger();

  uint32_t n = 42;
  std::string s = "Hello world";

  // Normal log macro
  AIMRT_HANDLE_LOG(lgr, aimrt::common::util::kLogLevelInfo, "This is a test log, n = {}, s = {}", n, s);
  AIMRT_HL_TRACE(lgr, "This is a test trace log, n = {}, s = {}", n, s);
  AIMRT_HL_DEBUG(lgr, "This is a test debug log, n = {}, s = {}", n, s);
  AIMRT_HL_INFO(lgr, "This is a test info log, n = {}, s = {}", n, s);
  AIMRT_HL_WARN(lgr, "This is a test warn log, n = {}, s = {}", n, s);
  AIMRT_HL_ERROR(lgr, "This is a test error log, n = {}, s = {}", n, s);
  AIMRT_HL_FATAL(lgr, "This is a test fatal log, n = {}, s = {}", n, s);

  // Check the expression and print the log only when it is false
  AIMRT_HL_CHECK_ERROR(lgr, n == 41, "Expression is not right, n = {}", n);

  // Print logs and throw exceptions
  AIMRT_HL_ERROR_THROW(lgr, "This is a test error log, n = {}, s = {}", n, s);

  // Check the expression, print the log and throw an exception when it is false
  AIMRT_HL_CHECK_TRACE_THROW(lgr, n == 41, "Expression is not right, n = {}", n);

  // ...
}
```


In addition, the logging component defines a default logger-handle acquisition interface `GetLogger()`. As long as the current context contains a `GetLogger()` method, you can use some more concise logging macros that implicitly use the result returned by `GetLogger()` as the logger handle, omitting the step of explicitly passing the logger handle. Example:

```cpp
#include "util/log_util.h"

auto GetLogger() {
  return aimrt::common::util::SimpleLogger();
}

int Main() {
  uint32_t n = 42;
  std::string s = "Hello world";

  // Normal log macro
  AIMRT_TRACE("This is a test trace log, n = {}, s = {}", n, s);
  AIMRT_DEBUG("This is a test debug log, n = {}, s = {}", n, s);
  AIMRT_INFO("This is a test info log, n = {}, s = {}", n, s);
  AIMRT_WARN("This is a test warn log, n = {}, s = {}", n, s);
  AIMRT_ERROR("This is a test error log, n = {}, s = {}", n, s);
  AIMRT_FATAL("This is a test fatal log, n = {}, s = {}", n, s);

  // Check the expression and print the log only when it is false
  AIMRT_CHECK_ERROR(n == 41, "Expression is not right, n = {}", n);

  // Print logs and throw exceptions
  AIMRT_ERROR_THROW("This is a test error log, n = {}, s = {}", n, s);

  // Check the expression, print the log and throw an exception when it is false
  AIMRT_CHECK_TRACE_THROW(n == 41, "Expression is not right, n = {}", n);

  // ...
}
```


## Runtime Logger Handle in AimRT

Within AimRT, a module can obtain an `aimrt::logger::LoggerRef` handle by calling the `GetLogger()` interface of the `CoreRef` handle. This class provides the `GetLogLevel` and `Log` interfaces, satisfying the requirements for a logger handle described in the previous section, and can be passed directly to the logging macros. Its core interface is as follows:

```cpp
namespace aimrt::logger {

class LoggerRef {
 public:
  // Get the log level
  uint32_t GetLogLevel() const;

  // Print logs
  void Log(uint32_t lvl, uint32_t line,
      const char* file_name, const char* function_name,
      const char* log_data, size_t log_data_size) const;
};

}  // namespace aimrt::logger
```


## Usage Examples of the Runtime Logger Handle in AimRT

Module developers can print logs using the logger handle assigned to the module by following the example below:

```cpp
#include "aimrt_module_cpp_interface/module_base.h"

class HelloWorldModule : public aimrt::ModuleBase {
 public:
  bool Initialize(aimrt::CoreRef core) override {
    logger_ = core_.GetLogger();

    uint32_t n = 42;
    std::string s = "Hello world";

    AIMRT_TRACE("This is a test trace log, n = {}, s = {}", n, s);
    AIMRT_DEBUG("This is a test debug log, n = {}, s = {}", n, s);
    AIMRT_INFO("This is a test info log, n = {}, s = {}", n, s);
    AIMRT_WARN("This is a test warn log, n = {}, s = {}", n, s);
    AIMRT_ERROR("This is a test error log, n = {}, s = {}", n, s);
    AIMRT_FATAL("This is a test fatal log, n = {}, s = {}", n, s);
  }

 private:
  auto GetLogger() { return logger_; }

 private:
  aimrt::logger::LoggerRef logger_;
};
```
