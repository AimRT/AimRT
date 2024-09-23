# Common Information


## Cpp 接口概览

AimRT 中的 C/Cpp 接口包括两部分：
1. 用于业务逻辑开发的接口，核心是`aimrt::CoreRef`句柄的使用；
2. 用于部署运行的接口，又分为 App 模式和 Pkg 模式，核心是如何获取`aimrt::CoreRef`句柄并区分 AimRT 运行阶段；


### Cpp 业务逻辑开发接口

AimRT 为业务逻辑开发提供了一套 CPP 接口库，CMake Target 名称：
- **aimrt::interface::aimrt_module_cpp_interface**

在开发业务逻辑时只需要链接这个 CMake Target，可以与 AimRT 的实现细节相隔离。这是一个纯头文件库，只有两个依赖：
- [fmt](https://github.com/fmtlib/fmt)：用于日志。如果编译器支持 C++20 的 format，可以关闭 CMake 选项`AIMRT_USE_FMT_LIB`，从而去掉这个依赖；
- [libunifex](https://github.com/facebookexperimental/libunifex)：用于将异步逻辑封装为协程；


该接口库核心是`aimrt::CoreRef`句柄，开发者可以通过此句柄来调用 AimRT 的各种能力，例如日志、RPC、Channel 等。关于`aimrt::CoreRef`句柄的相关信息，请参考[CoreRef](./core_ref.md)文档。


### CPP 部署运行接口


AimRT 提供了两种部署集成方式：
- **App 模式**：开发者在自己的 Main 函数中注册/创建各个模块，编译时直接将业务逻辑编译进主程序；
- **Pkg 模式**：使用 AimRT 提供的**aimrt_main**可执行程序，在运行时根据配置文件加载动态库形式的`Pkg`，导入其中的`Module`；


两者的核心区别是如何获取`aimrt::CoreRef`句柄并区分 AimRT 运行阶段。两者的优劣势和适用场景请参考[AimRT 中的基本概念](../concepts/concepts.md)文档里的说明。


关于这两种部署运行接口的详细信息，请参考[Runtime Interface](./runtime.md)文档。


## Cpp 接口中的引用类型

CPP 接口层中，大部分句柄都是一种引用类型，具有以下特点：
- 类型命名一般以`Ref`结尾；
- 这种引用类型一般需要一个 C 类型的指针来构造；
- 这种引用类型一般比较轻量，拷贝传递不会有较大开销；
- 这种引用类型一般都提供了一个`operator bool()`的重载来判断引用是否有效；
- 调用这种引用类型中提供的接口时，如果引用为空，会抛出一个异常；


## AimRT 生命周期以及接口调用时机

参考[接口概述](../concepts/interface.md)，AimRT 在运行时有三个主要的阶段：
- **Initialize** 阶段
- **Start** 阶段
- **Shutdown** 阶段

一些接口只能在某些特定阶段里被调用。后续文档中，如果不对接口做特殊说明，则默认该接口在所有阶段都可以被调用。


## 大部分接口的实际表现需要根据部署运行配置而定

在逻辑实现阶段，开发者只需要知道此接口在抽象意义上代表什么功能即可，至于在实际运行时的表现，则要根据部署运行配置而定，在逻辑开发阶段也不应该关心太多。

例如，开发者可以使用`log`接口打印一行日志，但这个日志最终打印到文件里还是控制台上，则需要根据运行时配置而定，开发者在写业务逻辑时不需要关心。


## Cpp 接口中的协程

AimRT 中为执行器、RPC 等功能提供了原生的异步回调形式的接口，同时也基于 C++20 协程和[C++ executors 提案](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p0443r14.html)当前的一个实现库[libunifex](https://github.com/facebookexperimental/libunifex)，为使用者提供了一套协程形式的接口。

关于协程接口的基本用法，将在执行器、RPC 等功能具体章节进行简单介绍。关于 C++20 协程以及 libunifex 库的进阶用法，请参考相关文档：
- [C++20 协程官方文档](https://en.cppreference.com/w/cpp/language/coroutines)
- [libunifex 主页](https://github.com/facebookexperimental/libunifex)


请注意，协程功能是一个 AimRT 框架中的一个可选项，如果使用者不想使用协程的方式，也仍然能够通过其他形式的接口使用 AimRT 框架的所有基础能力。


## Cpp 接口中的协议

在 **aimrt::interface::aimrt_module_cpp_interface** 这个 CMake Target 中，是不包含任何特定的协议类型的。在 AimRT 中，通过`aimrt_type_support_base_t`类来定义一种数据类型，其中定义了一种数据类型应该实现的基本接口，包括名称、创建/销毁、序列化/反序列化等。开发者可以通过实现它们来自定义数据类型，也可以直接使用AimRT官方支持的两种数据类型：
- **Protobuf**，需 CMake 引用 **aimrt::interface::aimrt_module_protobuf_interface**；
- **ROS2 Message**，需 CMake 引用 **aimrt::interface::aimrt_module_ros2_interface**；

一般在 Channel 或 RPC 功能中需要使用到具体的协议类型，具体的使用方式（包括代码生成、接口使用等）请参考 Channel 或 RPC 功能的文档章节。
