# Common Information

## C++ Interface Overview

The C/C++ interfaces in AimRT consist of two parts:
1. Interfaces for business logic development, with the core being the usage of the `aimrt::CoreRef` handle;
2. Interfaces for deployment and runtime, further divided into App mode and Pkg mode, focusing on how to obtain the `aimrt::CoreRef` handle and distinguish AimRT runtime phases;

### C++ Business Logic Development Interface

AimRT provides a set of C++ interface libraries for business logic development, with the CMake Target name:
- **aimrt::interface::aimrt_module_cpp_interface**

When developing business logic, you only need to link this CMake Target, which isolates you from AimRT's implementation details. This is a header-only library with only two dependencies:
- [fmt](https://github.com/fmtlib/fmt): Used for logging. If the compiler supports C++20's format, you can disable the CMake option `AIMRT_USE_FMT_LIB` to remove this dependency;
- [libunifex](https://github.com/facebookexperimental/libunifex): Used to encapsulate asynchronous logic as coroutines;

The core of this interface library is the `aimrt::CoreRef` handle. Developers can use this handle to access various capabilities of AimRT, such as logging, RPC, Channel, etc. For more information about the `aimrt::CoreRef` handle, please refer to the [CoreRef](./core_ref.md) documentation.

### C++ Deployment Runtime Interface

AimRT provides two deployment integration methods:
- **App Mode**: Developers register/create modules in their own main function, and the business logic is directly compiled into the main program;
- **Pkg Mode**: Uses the **aimrt_main** executable provided by AimRT, which loads `Pkg` in the form of dynamic libraries at runtime based on configuration files and imports the `Module` within them;

The core difference between the two lies in how to obtain the `aimrt::CoreRef` handle and distinguish AimRT runtime phases. For the advantages, disadvantages, and applicable scenarios of each, please refer to the explanations in the [Basic Concepts in AimRT](../concepts/concepts.md) documentation.

For detailed information about these two deployment runtime interfaces, please refer to the [Runtime Interface](./runtime.md) documentation.

## Reference Types in C++ Interfaces

In the C++ interface layer, most handles are reference types with the following characteristics:
- Type names generally end with `Ref`;
- These reference types usually require a C-type pointer for construction;
- These reference types are generally lightweight, and copying them does not incur significant overhead;
- These reference types usually provide an overloaded `operator bool()` to check if the reference is valid;
- When calling interfaces provided by these reference types, if the reference is null, an exception will be thrown;

## AimRT Lifecycle and Interface Call Timing

Refer to the [Interface Overview](../concepts/interface.md). AimRT has three main phases during runtime:
- **Initialize** phase
- **Start** phase
- **Shutdown** phase

Some interfaces can only be called during specific phases. In subsequent documentation, if no special note is made about an interface, it means the interface can be called in all phases by default.

## Actual Behavior of Most Interfaces Depends on Deployment Runtime Configuration

During the logic implementation phase, developers only need to understand what the interface represents in an abstract sense. The actual runtime behavior depends on the deployment runtime configuration, and developers should not concern themselves too much with this during the logic development phase.

For example, developers can use the `log` interface to print a line of logs, but whether the log is ultimately printed to a file or the console depends on the runtime configuration. Developers do not need to worry about this when writing business logic.

## Coroutines in C++ Interfaces

AimRT provides native asynchronous callback-style interfaces for executors, RPC, and other functionalities. At the same time, based on C++20 coroutines and an implementation library of the [C++ executors proposal](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p0443r14.html), [libunifex](https://github.com/facebookexperimental/libunifex), it provides a set of coroutine-style interfaces for users.

The basic usage of coroutine interfaces will be briefly introduced in the specific chapters on executors, RPC, and other functionalities. For advanced usage of C++20 coroutines and the libunifex library, please refer to the following documentation:
- [C++20 Coroutines Official Documentation](https://en.cppreference.com/w/cpp/language/coroutines)
- [libunifex Homepage](https://github.com/facebookexperimental/libunifex)

Note that coroutine functionality is an optional feature in the AimRT framework. If users prefer not to use coroutines, they can still access all the basic capabilities of the AimRT framework through other forms of interfaces.

## Protocols in C++ Interfaces

The CMake Target **aimrt::interface::aimrt_module_cpp_interface** does not include any specific protocol types. In AimRT, the `aimrt_type_support_base_t` class is used to define a data type, specifying the basic interfaces a data type should implement, including name, creation/destruction, serialization/deserialization, etc. Developers can implement these to define custom data types or directly use the two data types officially supported by AimRT:
- **Protobuf**, requiring CMake reference to **aimrt::interface::aimrt_module_protobuf_interface**;
- **ROS2 Message**, requiring CMake reference to **aimrt::interface::aimrt_module_ros2_interface**;

Generally, specific protocol types are needed in Channel or RPC functionalities. For specific usage methods (including code generation, interface usage, etc.), please refer to the documentation chapters on Channel or RPC functionalities.