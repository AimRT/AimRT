# Parameter

## Related Links

Code files:
- {{ '[aimrt_module_cpp_interface/parameter/parameter_handle.h]({}/src/interface/aimrt_module_cpp_interface/parameter/parameter_handle.h)'.format(code_site_root_path_url) }}

Reference examples:
- {{ '[parameter_module.cc]({}/src/examples/cpp/parameter/module/parameter_module/parameter_module.cc)'.format(code_site_root_path_url) }}


## Interface Overview

AimRT provides a simple module-level Key-Val parameter feature. Modules can obtain the `aimrt::parameter::ParameterHandleRef` handle by calling the `GetParameterHandle()` interface of the `CoreRef` handle to use this feature. The core interfaces provided by this handle are as follows:


```cpp
namespace aimrt::parameter {

class ParameterHandleRef {
 public:
  std::string GetParameter(std::string_view key) const;

  void SetParameter(std::string_view key, std::string_view val) const;
};

}  // namespace aimrt::parameter
```


Usage notes:
- The `std::string GetParameter(std::string_view key)` interface: used to get parameters.
  - If the key does not exist, an empty string is returned.
  - This interface is thread-safe.
- The `void SetParameter(std::string_view key, std::string_view val)` interface: used to set/update parameters.
  - If the key does not exist, a new key-val parameter pair is created.
  - If the key exists, the val value corresponding to the key is updated to the latest value.
  - This interface is thread-safe.
- Whether setting or getting parameters, they are module-level. Parameters of different modules are independent and invisible to each other.


In addition to setting/getting parameters through the parameter interface in CPP, users can also use the parameter_plugin to set/get parameters from external sources via HTTP and other methods. For details, please refer to the [parameter_plugin documentation](../plugins/parameter_plugin.md).

## Usage Example

A simple usage example is as follows:

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
