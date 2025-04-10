

# Common Information

## C++ Interface Overview

The C/C++ interfaces in AimRT consist of two parts:
1. Interfaces for business logic development, with the core being the use of the `aimrt::CoreRef` handle;
2. Interfaces for deployment and runtime, divided into App mode and Pkg mode, focusing on how to obtain the `aimrt::CoreRef` handle and distinguish AimRT runtime phases;

### C++ Business Logic Development Interface

AimRT provides a C++ interface library for business logic development, with CMake Target name:
- **aimrt::interface::aimrt_module_cpp_interface**

When developing business logic, you only need to link this CMake Target, which isolates implementation details from AimRT. This is a header-only library with two dependencies:
- [fmt](https://github.com/fmtlib/fmt): Used for logging. If the compiler supports C++20 format, you can disable the CMake option `AIMRT_USE_FMT_LIB` to remove this dependency;
- [libunifex](https://github.com/facebookexperimental/libunifex): Used to wrap asynchronous logic into coroutines;

The core of this interface library is the `aimrt::CoreRef` handle. Developers can use this handle to access various capabilities of AimRT such as logging, RPC, and Channels. For information about the `aimrt::CoreRef` handle, please refer to the [CoreRef](./core_ref.md) documentation.

### C++ Deployment Runtime Interface

AimRT provides two deployment integration methods:
- **App Mode**: Developers register/create modules in their own main function, compiling business logic directly into the main program;
- **Pkg Mode**: Uses the **aimrt_main** executable provided by AimRT, loading dynamic library-form `Pkg` modules at runtime according to configuration files;

The core difference lies in how to obtain the `aimrt::CoreRef` handle and distinguish AimRT runtime phases. For advantages/disadvantages and usage scenarios, please refer to the [Basic Concepts](../concepts/concepts.md) documentation.

For detailed information about these deployment interfaces, please see the [Runtime Interface](./runtime.md) documentation.

## Reference Types in C++ Interfaces

Most handles in the C++ interface layer are reference types with the following characteristics:
- Type names typically end with `Ref`;
- These reference types are usually constructed from C-style pointers;
- Lightweight with minimal copy overhead;
- Provide `operator bool()` overload to check validity;
- Calling interfaces on invalid references will throw an exception;

## AimRT Lifecycle and Interface Call Timing

As described in [Interface Overview](../concepts/interface.md), AimRT has three main runtime phases:
- **Initialize** phase
- **Start** phase
- **Shutdown** phase

Some interfaces can only be called during specific phases. Unless otherwise specified, interfaces are assumed to be available in all phases.

## Actual Behavior Depends on Runtime Configuration

During logic development, developers only need to understand the abstract functionality of interfaces. Actual runtime behavior depends on deployment configurations and should not concern business logic development.

For example, developers can use the `log` interface to print logs, but whether logs go to files or consoles is determined by runtime configuration.

## Coroutines in C++ Interfaces

AimRT provides native asynchronous callback interfaces for executors and RPC, while also offering coroutine-style interfaces based on C++20 coroutines and an implementation of [C++ executors proposal](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p0443r14.html) via [libunifex](https://github.com/facebookexperimental/libunifex).

Basic coroutine usage will be introduced in executor and RPC documentation chapters. For advanced usage, refer to:
- [C++20 Coroutines Reference](https://en.cppreference.com/w/cpp/language/coroutines)
- [libunifex Documentation](https://github.com/facebookexperimental/libunifex)


## Protocols in C++ Interfaces

The **aimrt::interface::aimrt_module_cpp_interface** CMake Target doesn't include specific protocol types. AimRT defines data types through `aimrt_type_support_base_t`, which specifies basic interfaces including name, creation/destruction, and serialization/deserialization.

Developers can implement custom types or use officially supported types:
- **Protobuf**: Requires CMake reference to **aimrt::interface::aimrt_module_protobuf_interface**;
- **ROS2 Message**: Requires CMake reference to **aimrt::interface::aimrt_module_ros2_interface**;

Protocol types are typically used in Channel or RPC functionalities. For usage details (including code generation), refer to respective documentation chapters.