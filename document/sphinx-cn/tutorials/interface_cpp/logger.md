# Logger

## 相关链接

代码文件：
- {{ '[util/log_util.h]({}/src/common/util/log_util.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/logger/logger.h]({}/src/interface/aimrt_module_cpp_interface/logger/logger.h)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[helloworld_module.cc]({}/src/examples/cpp/helloworld/module/helloworld_module/helloworld_module.cc)'.format(code_site_root_path_url) }}


## AimRT 中的独立日志组件

AimRT 提供了一个独立的通用日志组件，属于 **aimrt::common::util** 这个CMake Target，只需要`#include "util/log_util.h"`即可独立于 CPP 接口层使用。

其中提供了一些基础的日志宏，这些日志宏需要在调用时传入一个`Logger`对象，来定义日志打印行为的具体表现。日志句柄以模板 Concept 的形式定义，只要是类似于以下这个示例、包含`GetLogLevel`和`Log`两个接口的 C++ 类的实例都可以作为日志句柄：

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

其中，日志等级分为以下 6 档：
- Trace
- Debug
- Info
- Warn
- Error
- Fatal

在有了日志句柄之后，开发者可以直接基于日志句柄提供的`Log`方法打印日志，也可以使用提供的日志宏来更方便的打印日志。注意，提供的日志宏基于 C++20 Format 语法，关于 C++20 Format 语法的详细使用方式请参考[C++官方文档](https://en.cppreference.com/w/cpp/utility/format)。

AimRT 在`util/log_util.h`文件中，还提供了两种默认的`Logger`类型：
- **SimpleLogger** ：一个简单的同步日志句柄；
- **SimpleAsyncLogger** ： 一个简单的异步日志句柄；


这两种日志句柄一般用于单元测试等未启动 AimRT 实例时的场景。


## AimRT 中的独立日志组件使用示例

以下是一些使用示例：
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

此外，日志组件中还定义了一个默认的日志句柄获取接口`GetLogger()`，只要当前上下文中有`GetLogger()`这个方法，即可使用一些更简洁的日志宏，隐式的将`GetLogger()`方法返回的结果作为日志句柄，省略掉显式传递日志句柄这一步。示例如下：
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

## AimRT 运行时日志句柄


在 AimRT 中，模块可以通过调用`CoreRef`句柄的`GetLogger()`接口，获取`aimrt::logger::LoggerRef`句柄，这是一个包含`GetLogLevel`和`Log`接口的类，满足上一节中对日志句柄的要求，可以直接作为日志宏的参数。其核心接口如下：
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

## AimRT 运行时日志句柄使用示例

模块开发者可以直接参照以下示例的方式，使用分配给模块的日志句柄来打印日志：
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
