

# Parameter

## Related Links

Code Files:
- {{ '[aimrt_module_cpp_interface/parameter/parameter_handle.h]({}/src/interface/aimrt_module_cpp_interface/parameter/parameter_handle.h)'.format(code_site_root_path_url) }}

Reference Examples:
- {{ '[parameter_module.cc]({}/src/examples/cpp/parameter/module/parameter_module/parameter_module.cc)'.format(code_site_root_path_url) }}

## Interface Overview

AimRT provides a simple module-level Key-Val parameter functionality. Modules can obtain the `aimrt::parameter::ParameterHandleRef` handle by calling the `GetParameterHandle()` interface of the `CoreRef` handle to use this feature. The core interfaces provided by this handle are as follows:

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
- `std::string GetParameter(std::string_view key)` interface: Used to retrieve parameters.
  - Returns an empty string if the key doesn't exist.
  - This interface is thread-safe.
- `void SetParameter(std::string_view key, std::string_view val)` interface: Used to set/update parameters.
  - Creates a new key-val pair if the key doesn't exist.
  - Updates the val corresponding to the key if it exists.
  - This interface is thread-safe.
- Both setting and getting parameters are module-level operations. Parameters between different modules remain independent and invisible to each other.

In addition to setting/getting parameters through the CPP interface, users can also use the parameter_plugin to set/get parameters externally via HTTP. For details, please refer to the [parameter_plugin documentation](../plugins/parameter_plugin.md).

## Usage Example

A simple usage example is shown below:
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