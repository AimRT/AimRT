# Basic Concepts in AimRT

## Contents Included in the AimRT Framework

Referring to the `src` directory in the AimRT source code, the contents included in the AimRT framework are as follows:
```
src
├── common --------------------------------- // Some basic, common components that can be used directly, such as string, log interface, buffer, etc.
├── examples ------------------------------- // AimRT official examples
│   ├── cpp -------------------------------- // Examples of CPP interface
│   ├── py --------------------------------- // Examples of Python interface
│   └── plugins ---------------------------- // Examples of plugins
├── interface ------------------------------ // AimRT Interface Layer
│   ├── aimrt_core_plugin_interface -------- // [CPP] Plugin development interface
│   ├── aimrt_module_c_interface ----------- // [C] Module development interface
│   ├── aimrt_module_cpp_interface --------- // [CPP] Module development interface, encapsulation of the C version
│   ├── aimrt_module_protobuf_interface ---- // [CPP] Module development interface related to protobuf, based on CPP version interface
│   ├── aimrt_module_ros2_interface -------- // [CPP] Module development interface related to ROS2, based on CPP version interface
│   ├── aimrt_pkg_c_interface -------------- // [C] Pkg development interface
│   └── aimrt_type_support_pkg_c_interface - // [C] Type support package interface
├── plugins -------------------------------- // AimRT official plugin
├── protocols ------------------------------ // AimRT official standard protocols
├── runtime -------------------------------- // AimRT runtime
│   ├── core ------------------------------- // Runtime core library
│   ├── main ------------------------------- // A main process "aimrt_main" based on core
│   └── python_runtime --------------------- // Python version runtime based on pybind11
└── tools ---------------------------------- // Supporting tools
```

## The "Module" Concept in AimRT
Like most frameworks, AimRT has a concept called `Module` to identify independent logical units. A `Module` is a logical-level concept representing a cohesive block of logic. `Modules` can communicate with each other at the logical level through two abstract interfaces: `Channel` and `RPC`. Creating a `Module` involves implementing a few simple interfaces.

A `Module` typically corresponds to a hardware abstraction, an independent algorithm, or a business function. A `Module` can use handles provided by the framework to invoke various runtime functionalities, such as configuration, logging, and executors. The framework provides independent handles for each `Module` to enable resource statistics and management features.

## The "Node" Concept in AimRT
A `Node` represents a deployable and executable process that runs an instance of the AimRT framework's Runtime. A `Node` is a deployment and runtime-level concept, and a single `Node` may contain multiple `Modules`. During startup, a `Node` can configure runtime parameters such as logging, plugins, and executors via configuration files.

## The "Pkg" Concept in AimRT
`Pkg` is a way for the AimRT framework to run `Modules`. A `Pkg` represents a dynamic library containing one or multiple `Modules`, and a `Node` can load one or more `Pkgs` at runtime. Creating a `Pkg` involves implementing a few simple module description interfaces.

The `Module` concept is more focused on the code logic level, while `Pkg` is a deployment-level concept that does not contain business logic code. Generally, when compatibility allows, it is recommended to compile multiple `Modules` into a single `Pkg`, as this optimizes performance when using features like RPC and Channel.

Typically, symbols in a `Pkg` are hidden by default, exposing only a limited set of pure C interfaces. Different `Pkgs` do not interfere with each other's symbols. Theoretically, different `Pkgs` can be compiled independently using different compiler versions, and `Modules` within different `Pkgs` can use conflicting third-party dependencies. The compiled `Pkgs` can be distributed as binaries.

## Two Ways to Integrate Business Logic in AimRT
The AimRT framework supports two methods for integrating business logic: **App Mode** and **Pkg Mode**. The choice between them depends on the specific scenario. Their differences are as follows:
- **App Mode**: Directly link the AimRT runtime library in the developer's own `main` function, compiling business logic code directly into the main executable:
  - **Advantages**: No `dlopen` step or shared objects (`so`), resulting in a single executable (`exe`).
  - **Disadvantages**: Potential conflicts with third-party libraries; inability to independently release `Modules` (only the executable can be distributed).
  - **Use Case**: Suitable for small tools or demo scenarios with minimal module decoupling requirements.
- **Pkg Mode**: Use the **aimrt_main** executable provided by AimRT, loading `Pkgs` (as dynamic libraries) at runtime based on configuration files and importing their `Module` classes:
  - **Advantages**: Only a lightweight AimRT interface layer needs to be linked when compiling business `Modules`, avoiding potential dependency conflicts; supports binary distribution of `so` files; better independence.
  - **Disadvantages**: The framework loads `Pkgs` via `dlopen`, which may cause compatibility issues in rare cases.
  - **Use Case**: Suitable for medium to large projects with strong requirements for module decoupling and binary distribution.

Both methods can coexist without affecting business logic, and the choice depends on the specific scenario.


## The "Protocol" Concept in AimRT
`Protocol` refers to the data format for communication between `Modules`, describing field information and serialization/deserialization methods. For example, it defines the data format agreed upon between `Channel` publishers and subscribers or between `RPC` clients and servers. Typically, an `IDL` (Interface Description Language) is used to describe the protocol, which is then converted into code for various languages using specific tools.

AimRT officially supports two IDLs:
- Protobuf
- ROS2 msg/srv

However, AimRT does not restrict the type of protocol or IDL. Users can implement other IDLs, such as Thrift IDL, FlatBuffers, or even custom IDLs.

## The "Channel" Concept in AimRT
`Channel`, also known as a data channel, is a typical communication topology concept. It identifies a single data channel via a `Topic` and consists of `Publishers` and `Subscribers`, where subscribers receive data published by publishers. A `Channel` is a many-to-many topology structure: a `Module` can publish data to any number of `Topics` and subscribe to any number of `Topics`. Similar concepts include ROS Topics and message queues like Kafka/RabbitMQ.

In AimRT, a Channel consists of two decoupled parts: the `Interface Layer` and the `Backend`. The interface layer defines an abstract API representing the logical-level `Channel`, while the backend handles actual data transmission and can be of various types. AimRT officially provides several Channel backends, such as MQTT and ROS2, but users can also develop their own.

When using AimRT's Channel feature, developers first call the interface layer API in the business logic layer to publish data to a `Topic` or subscribe to a `Topic`. The AimRT framework then selects one or more Channel backends based on specific rules to process the data. These backends transmit the data to other nodes, where corresponding Channel backends receive and deliver the data to the business logic layer. The workflow is illustrated below:

![](./picture/pic_3.png)

## The "RPC" Concept in AimRT
`RPC` (Remote Procedure Call) is based on a request-reply model and consists of `Clients` and `Servers`. A `Module` can create a client handle to initiate specific RPC requests, which are received and replied to by a designated server or one selected by the framework based on rules. A `Module` can also create a server handle to provide specific RPC services, processing and replying to routed requests. Similar concepts include ROS Services and RPC frameworks like gRPC/Thrift.

In AimRT, RPC also consists of two decoupled parts: the `Interface Layer` and the `Backend`. The interface layer defines the abstract RPC API, while the backend handles the actual RPC calls. AimRT officially provides several RPC backends, such as HTTP and ROS2, but users can develop their own.

When using AimRT's RPC feature, developers call the interface layer API in the business logic layer to initiate an RPC call via a `Client`. The AimRT framework selects an RPC backend based on specific rules to process the data. The backend transmits the data to the server node, where the corresponding RPC backend receives and delivers it to the business layer. The reply is then sent back to the client. The workflow is illustrated below:

![](./picture/pic_4.png)

## The "Filter" Concept in AimRT
AimRT provides a `Filter` feature to enhance RPC or Channel capabilities. A Filter is a user-customizable logic insertion point adjacent to the Interface Layer. Relative to the Interface Layer, Filters are categorized as Framework Filters and User Filters. By functionality, they are further divided into RPC Filters and Channel Filters.

Filters are triggered during each RPC or Channel call, operating in an onion-like structure to perform custom actions before or after the call, such as measuring latency or reporting metrics. The workflow for an RPC Filter is illustrated below:

![RPC Filter](./picture/pic_7.png)

## The "Executor" Concept in AimRT
An `Executor`, or task runner, is an abstract concept representing an entity capable of executing tasks. An executor can be a Fiber, Thread, or Thread Pool. By default, the code we write specifies an executor: the main thread. Generally, any entity providing the following interface can be considered an executor:
```cpp
void Execute(std::function<void()>&& task);
```

Another type of `Executor` supports scheduled execution, allowing tasks to be executed at a specific time or after a delay. Its interface resembles the following:
```cpp
void ExecuteAt(std::chrono::system_clock::time_point tp, std::function<void()>&& task);
void ExecuteAfter(std::chrono::nanoseconds dt, std::function<void()>&& task);
```

In AimRT, the executor functionality consists of two decoupled parts: the **Interface Layer** and the **Executor Implementation**. The interface layer defines the abstract executor API for submitting tasks, while the implementation layer handles actual task execution, with behavior varying by implementation type. AimRT officially provides several executors, such as an Asio-based thread pool, a Tbb-based lock-free thread pool, and a timer-wheel-based scheduled executor.

When using AimRT's executor feature, developers package tasks into closures in the business layer and call the interface layer API to submit them to a specific executor. The executor then schedules and executes the tasks based on its own policies. The workflow is illustrated below:

![](./picture/pic_5.png)

## The "Plugin" Concept in AimRT
A `Plugin` refers to a dynamic library that can register custom functionalities with the AimRT framework. Plugins can be loaded at runtime by the framework or hardcoded into user-defined executables. The AimRT framework exposes numerous plugin points and query interfaces, such as:
- Logging backend registration interface
- Channel/RPC backend registration interface
- Channel/RPC registry query interface
- Component startup hook points
- RPC/Channel call filters
- Module information query interface
- Executor registration interface
- Executor query interface
- ......

Users can directly use official AimRT plugins, seek third-party plugins, or implement their own to extend the framework's capabilities for specific needs.