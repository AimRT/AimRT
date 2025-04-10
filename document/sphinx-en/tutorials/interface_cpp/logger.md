

# Logger

## Related Links

Code Files:
- {{ '[util/log_util.h]({}/src/common/util/log_util.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/logger/logger.h]({}/src/interface/aimrt_module_cpp_interface/logger/logger.h)'.format(code_site_root_path_url) }}

Reference Example:
- {{ '[helloworld_module.cc]({}/src/examples/cpp/helloworld/module/helloworld_module/helloworld_module.cc)'.format(code_site_root_path_url) }}

## Standalone Logging Component in AimRT

AimRT provides an independent general-purpose logging component that belongs to the **aimrt::common::util** CMake Target. Simply use `#include "util/log_util.h"` to use it independently of the CPP interface layer.

It provides some basic logging macros that require passing a `Logger` object when called to define the specific behavior of log printing. The log handle is defined in the form of a template Concept - any C++ class instance containing both `GetLogLevel` and `Log` interfaces, similar to the following example, can serve as a log handle:

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

The log levels are divided into the following 6 tiers:
- Trace
- Debug
- Info
- Warn
- Error
- Fatal

After obtaining a log handle, developers can either directly use the `Log` method provided by the log handle to print logs, or use the provided logging macros for more convenient logging. Note that the provided macros are based on C++20 Format syntax. For detailed usage of C++20 Format syntax, please refer to the [C++ Official Documentation](https://en.cppreference.com/w/cpp/utility/format).

AimRT also provides two default `Logger` types in the `util/log_util.h` file:
- **SimpleLogger**: A simple synchronous log handle;
- **SimpleAsyncLogger**: A simple asynchronous log handle;

These two log handles are typically used in scenarios where the AimRT instance is not started, such as unit testing.

## Usage Examples of Standalone Logging Component

Here are some usage examples:
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

Additionally, the logging component defines a default log handle acquisition interface `GetLogger()`. As long as there is a `GetLogger()` method in the current context, more concise logging macros can be used to implicitly take the result returned by `GetLogger()` as the log handle, omitting the step of explicitly passing the log handle. Example:
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

## Runtime Log Handles in AimRT

In AimRT, modules can obtain the `aimrt::logger::LoggerRef` handle by calling the `GetLogger()` interface of the `CoreRef` handle. This is a class containing both `GetLogLevel` and `Log` interfaces, meeting the requirements for log handles described in the previous section, and can be directly used as parameters for logging macros. Its core interfaces are as follows:
```cpp
namespace aimrt::logger {

class LoggerRef {
 public:
  // 获取日志等级
  uint32_t GetLogLevel() const;

  // 打印日志
  void Log(uint32_t lvl, uint32_t line,
      const char* file_name, const char* function_name,
      const char* log_data, size_t log_data_size) const;
};

}  // namespace aimrt::logger
```

## Runtime Log Handle Usage Examples

Module developers can directly refer to the following example to use the log handle assigned to the module for logging:
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