# Parameter

## Related Links

Code Files:
- {{ '[aimrt_module_cpp_interface/parameter/parameter_handle.h]({}/src/interface/aimrt_module_cpp_interface/parameter/parameter_handle.h)'.format(code_site_root_path_url) }}

Reference Examples:
- {{ '[parameter_module.cc]({}/src/examples/cpp/parameter/module/parameter_module/parameter_module.cc)'.format(code_site_root_path_url) }}

## Interface Overview

AimRT provides a simple module-level Key-Val parameter feature. Modules can obtain the `aimrt::parameter::ParameterHandleRef` handle by calling the `GetParameterHandle()` interface of the `CoreRef` handle to use this functionality. The core interfaces provided by this handle are as follows:

```cpp
namespace aimrt::parameter {

class ParameterHandleRef {
 public:
  std::string GetParameter(std::string_view key) const;

  void SetParameter(std::string_view key, std::string_view val) const;
};

}  // namespace aimrt::parameter
```

Usage Notes:
- `std::string GetParameter(std::string_view key)` interface: Used to retrieve parameters.
  - Returns an empty string if the key does not exist.
  - This interface is thread-safe.
- `void SetParameter(std::string_view key, std::string_view val)` interface: Used to set/update parameters.
  - Creates a new key-val pair if the key does not exist.
  - Updates the corresponding val value if the key exists.
  - This interface is thread-safe.
- Both setting and retrieving parameters are module-level operations. Parameters of different modules are independent and invisible to each other.

In addition to using the parameter interfaces in the CPP interface to set/retrieve parameters, users can also utilize the parameter_plugin to set/retrieve parameters externally via HTTP. For details, please refer to the [parameter_plugin documentation](../plugins/parameter_plugin.md).

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