

# Basic Concepts in AimRT

## Contents of the AimRT Framework

Referencing the src directory in the AimRT source code, the framework contains:
```
src
├── common --------------------------------- // 一些基础的、可以直接使用的通用组件，例如 string、log 接口、buffer 等
├── examples ------------------------------- // AimRT 官方示例
│   ├── cpp -------------------------------- // CPP 接口的示例
│   ├── py --------------------------------- // Python 接口的示例
│   └── plugins ---------------------------- // 一些各方插件的使用示例
├── interface ------------------------------ // AimRT 接口层
│   ├── aimrt_core_plugin_interface -------- // [CPP] 插件开发接口
│   ├── aimrt_module_c_interface ----------- // [C] 模块开发接口
│   ├── aimrt_module_cpp_interface --------- // [CPP] 模块开发接口，对 C 版本的封装
│   ├── aimrt_module_protobuf_interface ---- // [CPP] 与 protobuf 相关的模块开发接口，基于 CPP 版本接口
│   ├── aimrt_module_ros2_interface -------- // [CPP] 与 ROS2 相关的模块开发接口，基于 CPP 版本接口
│   ├── aimrt_pkg_c_interface -------------- // [C] Pkg 开发接口
│   └── aimrt_type_support_pkg_c_interface - // [C] Type support 包接口
├── plugins -------------------------------- // AimRT 官方插件
├── protocols ------------------------------ // 一些 AimRT 官方的标准协议
├── runtime -------------------------------- // AimRT 运行时
│   ├── core ------------------------------- // 运行时核心库
│   ├── main ------------------------------- // 基于 core 实现的一个主进程"aimrt_main"
│   └── python_runtime --------------------- // 基于 pybind11 封装的 python 版本运行时
└── tools ---------------------------------- // 一些配套工具
```

## The "Module" Concept in AimRT
Like most frameworks, AimRT has a concept for identifying independent logical units: `Module`. A `Module` is a logical-level concept representing a cohesive logical block. Modules can communicate through two abstract interfaces at the logical layer: `Channel` and `RPC`. Developers can create modules by implementing several simple interfaces.

A `Module` typically corresponds to hardware abstraction, an independent algorithm, or a business function. Modules can use framework-provided handles to access runtime features like configuration, logging, and executors. The framework provides independent handles to each module for resource statistics and management purposes.

## The "Node" Concept in AimRT
`Node` represents a deployable process running an instance of AimRT's runtime. It is a deployment/operation-level concept where a single Node may contain multiple Modules. Nodes can configure runtime parameters like logging, plugins, and executors through configuration files during startup.

## The "Pkg" Concept in AimRT
`Pkg` is AimRT's approach to running Modules. It represents a dynamic library containing one or multiple Modules, which Nodes can load during runtime. Developers can create Pkgs by implementing simple module description interfaces.

While Modules focus on code logic, Pkgs are deployment-level concepts without business logic code. For compatibility, it's recommended to compile multiple Modules into a single Pkg for RPC and Channel performance optimizations.

Typically, Pkgs expose only limited pure C interfaces with hidden symbols, avoiding cross-pkg symbol conflicts. Different Pkgs can be compiled independently with different compiler versions and conflicting third-party dependencies, enabling binary distribution.

## Two Integration Modes for Business Logic
AimRT supports two integration approaches:
- **App Mode**: Directly links AimRT runtime library in developer's main function, compiling business logic into executable:
  - **Advantages**: No dlopen step; single executable output
  - **Disadvantages**: Potential third-party library conflicts; no independent Module distribution
  - **Use Case**: Small tools/demos with minimal modularity requirements
- **Pkg Mode**: Uses AimRT's **aimrt_main** executable to load Pkgs (dynamic libraries) at runtime via configuration:
  - **Advantages**: Lightweight interface layer linking; binary SO distribution; better isolation
  - **Disadvantages**: Rare dlopen compatibility issues
  - **Use Case**: Medium/large projects requiring modular decoupling

Both modes coexist without affecting business logic. Python development only supports App Mode.

## The "Protocol" Concept in AimRT
`Protocol` defines data formats for Module communication, describing field information and serialization methods. Supported IDLs include:
- Protobuf
- ROS2 msg/srv

AimRT doesn't restrict protocol types - users can implement custom IDLs like Thrift or FlatBuffers.

## The "Channel" Concept in AimRT
`Channel` implements a pub-sub pattern using `Topic` identifiers. It features:
- Multi-publisher/multi-subscriber topology
- Decoupled interface layer and backend implementation
- Official backends: MQTT, ROS2, etc.
- Custom backend development support

![](./picture/pic_3.png)

## The "RPC" Concept in AimRT
`RPC` implements request-response communication with:
- Client-server model
- Decoupled interface layer and backend
- Official backends: HTTP, ROS2, etc.
- Custom backend support

![](./picture/pic_4.png)

## The "Filter" Concept in AimRT
`Filter` enhances RPC/Channel capabilities through:
- Framework-side and user-side filters
- Onion-style execution flow
- Capabilities like latency monitoring

![RPC Filter](./picture/pic_7.png)

## The "Executor" Concept in AimRT
`Executor` abstracts task execution with:
- Interface layer and implementation separation
- Support for thread pools, fibers, etc.
- Timer-based execution capabilities

Standard interface:
```cpp
void Execute(std::function<void()>&& task);
```

Timed execution interface:
```cpp
void ExecuteAt(std::chrono::system_clock::time_point tp, std::function<void()>&& task);
void ExecuteAfter(std::chrono::nanoseconds dt, std::function<void()>&& task);
```

![](./picture/pic_5.png)

## The "Plugin" Concept in AimRT
`Plugin` extends framework capabilities through:
- Dynamic library registration
- Extensive extension points:
  - Logging backends
  - Communication backends
  - Hook points
  - Filter systems
  - Module queries
  - Executor management

Supports official/third-party/custom plugin implementations.