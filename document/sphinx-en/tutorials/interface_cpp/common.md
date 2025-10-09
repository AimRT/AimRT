# Common Information


## Cpp Interface Overview

The C/Cpp interfaces in AimRT consist of two parts:
1. Interfaces for business logic development, with the core being the use of the `aimrt::CoreRef` handle;
2. Interfaces for deployment and runtime, further divided into App mode and Pkg mode, focusing on how to obtain the `aimrt::CoreRef` handle and distinguish AimRT runtime phases;


### Cpp Business Logic Development Interface

AimRT provides a CPP interface library for business logic development, with the CMake Target name:
- **aimrt::interface::aimrt_module_cpp_interface**

When developing business logic, you only need to link this CMake Target, which isolates you from AimRT's implementation details. This is a header-only library with only two dependencies:
- [fmt](https://github.com/fmtlib/fmt): Used for logging. If the compiler supports C++20's format, you can turn off the CMake option `AIMRT_USE_FMT_LIB` to remove this dependency;
- [libunifex](https://github.com/facebookexperimental/libunifex): Used to encapsulate asynchronous logic as coroutines;


The core of this interface library is the `aimrt::CoreRef` handle, through which developers can invoke various AimRT capabilities such as logging, RPC, Channel, etc. For information about the `aimrt::CoreRef` handle, please refer to the [CoreRef](./core_ref.md) documentation.


### CPP Deployment and Runtime Interface


AimRT provides two deployment integration methods:
- **App mode**: Developers register/create various modules in their own Main function, compiling business logic directly into the main program at build time;
- **Pkg mode**: Uses the **aimrt_main** executable provided by AimRT, loading dynamic library form `Pkg`s at runtime based on configuration files, importing `Module`s within them;


The core difference between the two lies in how to obtain the `aimrt::CoreRef` handle and distinguish AimRT runtime phases. For their advantages, disadvantages, and applicable scenarios, please refer to the explanations in the [Basic Concepts in AimRT](../concepts/concepts.md) documentation.


For detailed information about these two deployment runtime interfaces, please refer to the [Runtime Interface](./runtime.md) documentation.


## Reference Types in Cpp Interface

In the CPP interface layer, most handles are reference types with the following characteristics:
- Type names generally end with `Ref`;
- These reference types generally require a C-style pointer for construction;
- These reference types are generally lightweight, with copying and passing incurring minimal overhead;
- These reference types generally provide an `operator bool()` overload to check if the reference is valid;
- When calling interfaces provided by these reference types, if the reference is null, an exception will be thrown;


## AimRT Lifecycle and Interface Call Timing

Refer to [Interface Overview](../concepts/interface.md), AimRT has three main phases during runtime:
- **Initialize** phase
- **Start** phase
- **Shutdown** phase

Some interfaces can only be called during specific phases. In subsequent documentation, unless specifically noted, interfaces are assumed to be callable in all phases.


## Actual Behavior of Most Interfaces Depends on Deployment Configuration

During the logic implementation phase, developers only need to know what function an interface represents in an abstract sense. The actual runtime behavior depends on the deployment configuration, and developers should not concern themselves too much during logic development.

For example, developers can use the `log` interface to print a line of log, but whether this log ultimately prints to a file or the console depends on the runtime configuration, which developers don't need to worry about when writing business logic.


## Coroutines in Cpp Interface

AimRT provides native asynchronous callback-style interfaces for executors, RPC, and other functions. It also offers a set of coroutine-style interfaces based on C++20 coroutines and [C++ executors proposal](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p0443r14.html) implementation library [libunifex](https://github.com/facebookexperimental/libunifex).

For basic usage of coroutine interfaces, a brief introduction will be provided in specific chapters about executors, RPC, and other functions. For advanced usage of C++20 coroutines and the libunifex library, please refer to related documentation:
- [C++20 Coroutines Official Documentation](https://en.cppreference.com/w/cpp/language/coroutines)
- [libunifex Homepage](https://github.com/facebookexperimental/libunifex)


Please note that coroutine functionality is an optional feature in the AimRT framework. If users do not wish to use coroutines, they can still use all basic capabilities of the AimRT framework through other interface forms.


## Protocols in Cpp Interface

The **aimrt::interface::aimrt_module_cpp_interface** CMake Target does not include any specific protocol types. In AimRT, a data type is defined through the `aimrt_type_support_base_t` class, which defines the basic interfaces a data type should implement, including name, creation/destruction, serialization/deserialization, etc. Developers can implement these to customize data types, or directly use the two data types officially supported by AimRT:
- **Protobuf**, requiring CMake reference to **aimrt::interface::aimrt_module_protobuf_interface**;
- **ROS2 Message**, requiring CMake reference to **aimrt::interface::aimrt_module_ros2_interface**;

Specific protocol types are generally needed in Channel or RPC functions. For specific usage methods (including code generation, interface usage, etc.), please refer to the documentation chapters for Channel or RPC functions.