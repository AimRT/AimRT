# Parameter

## 相关链接

代码文件：
- {{ '[aimrt_module_cpp_interface/parameter/parameter_handle.h]({}/src/interface/aimrt_module_cpp_interface/parameter/parameter_handle.h)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[parameter_module.cc]({}/src/examples/cpp/parameter/module/parameter_module/parameter_module.cc)'.format(code_site_root_path_url) }}


## 接口概述

AimRT 中提供了一个简单的模块级 Key-Val 参数功能，模块可以通过调用`CoreRef`句柄的`GetParameterHandle()`接口，获取`aimrt::parameter::ParameterHandleRef`句柄，来使用此功能。该句柄提供的核心接口如下：

```cpp
namespace aimrt::parameter {

class ParameterHandleRef {
 public:
  std::string GetParameter(std::string_view key) const;

  void SetParameter(std::string_view key, std::string_view val) const;
};

}  // namespace aimrt::parameter
```

使用注意点如下：
- `std::string GetParameter(std::string_view key)`接口：用于获取参数。
  - 如果不存在 key，则返回空字符串。
  - 该接口是线程安全的。
- `void SetParameter(std::string_view key, std::string_view val)`接口：用于设置/更新参数。
  - 如果不存在 key，则新建一个 key-val 参数对。
  - 如果存在 key，则更新 key 所对应的 val 值为最新值。
  - 该接口是线程安全的。
- 无论是设置参数还是获取参数，都是模块级别的，不同模块的参数互相独立、互不可见。


除了通过 CPP 接口中的参数接口来设置/获取参数，使用者也可以通过 parameter_plugin，通过 HTTP 等方式来从外部设置/获取参数。具体请参考[parameter_plugin 文档](../plugins/parameter_plugin.md)。

## 使用示例

一个简单的使用示例如下：
```cpp
class HelloWorldModule : public aimrt::ModuleBase {
 public:
  bool Initialize(aimrt::CoreRef core) override {
    auto parameter_handle = core.GetParameterHandle();

    std::string key = "test key";
    std::string val = "test val";

    // Set
    parameter_handle.SetParameter(key, val);

    // Get
    std::string check_val = parameter_handle.GetParameter(key);

    assert(val == check_val);

    return true;
  }
};
```

