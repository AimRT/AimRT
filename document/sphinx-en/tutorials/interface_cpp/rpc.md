

# RPC

## Related Links

Code Files:
- {{ '[aimrt_module_cpp_interface/rpc/rpc_handle.h]({}/src/interface/aimrt_module_cpp_interface/rpc/rpc_handle.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/rpc/rpc_context.h]({}/src/interface/aimrt_module_cpp_interface/rpc/rpc_context.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_c_interface/rpc/rpc_status_base.h]({}/src/interface/aimrt_module_c_interface/rpc/rpc_status_base.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/rpc/rpc_status.h]({}/src/interface/aimrt_module_cpp_interface/rpc/rpc_status.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/rpc/rpc_co_filter.h]({}/src/interface/aimrt_module_cpp_interface/rpc/rpc_co_filter.h)'.format(code_site_root_path_url) }}

Reference Examples:
- {{ '[pb_rpc]({}/src/examples/cpp/pb_rpc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc]({}/src/examples/cpp/ros2_rpc)'.format(code_site_root_path_url) }}

## Protocol

Protocols are used to determine the message format between clients and servers in RPC. Generally, protocols are described using an IDL (Interface Description Language) that is programming language agnostic, then converted to code for various languages using specific tools. For RPC, this process involves two steps:
- As described in the [Channel](./channel.md) chapter, developers first need to use official tools to generate code for **message types** defined in protocol files for target programming languages;
- Developers need to use tools provided by AimRT to generate code for **service definitions** in protocol files for target programming languages;

### Protobuf

[Protobuf](https://protobuf.dev/) is a lightweight, efficient data interchange format developed by Google for serializing structured data, and is a widely used IDL. It can not only describe message structures but also define RPC services using `service` statements.

When using Protobuf, developers first need to define a `.proto` file containing message structures and RPC services. For example `rpc.proto`:

```protobuf
syntax = "proto3";

message ExampleReq {
  string msg = 1;
  int32 num = 2;
}

message ExampleRsp {
  uint64 code = 1;
  string msg = 2;
}

service ExampleService {
  rpc ExampleFunc(ExampleReq) returns (ExampleRsp);
}
```

Then use the official protoc tool from Protobuf to generate C++ code for message structures:
```shell
protoc --cpp_out=. rpc.proto
```

This generates `rpc.pb.h` and `rpc.pb.cc` files containing C++ classes and methods for the defined message types.

Next, use the protoc plugin provided by AimRT to generate C++ code for service definitions:
```shell
protoc --aimrt_rpc_out=. --plugin=protoc-gen-aimrt_rpc=./protoc_plugin_py_gen_aimrt_cpp_rpc.py rpc.proto
```

This generates `rpc.aimrt_rpc.pb.h` and `rpc.aimrt_rpc.pb.cc` files containing C++ classes and methods for the defined services.

Note that this native code generation approach mainly demonstrates underlying principles. Practical usage requires manual handling of dependencies and CMake configurations, which can be cumbersome. AimRT provides encapsulated solutions through these two CMake files:

1. {{ '[ProtobufGenCode.cmake]({}/cmake/ProtobufGenCode.cmake)'.format(code_site_root_path_url) }}: Generates C++ code for message structures, containing two CMake methods:
- `add_protobuf_gencode_target_for_proto_path`: Generates C++ code for `.proto` files in specified path with parameters:
  - **TARGET_NAME**: Generated CMake target name
  - **PROTO_PATH**: Protocol file directory
  - **GENCODE_PATH**: Generated code output path
  - **DEP_PROTO_TARGETS**: Dependent Proto CMake targets
  - **OPTIONS**: Additional protoc arguments
- `add_protobuf_gencode_target_for_one_proto_file`: Generates C++ code for single `.proto` file with parameters:
  - **TARGET_NAME**: Generated CMake target name
  - **PROTO_FILE**: Path to single protocol file
  - **GENCODE_PATH**: Generated code output path
  - **DEP_PROTO_TARGETS**: Dependent Proto CMake targets
  - **OPTIONS**: Additional protoc arguments

2. {{ '[ProtobufAimRTRpcGenCode.cmake]({}/src/tools/protoc_plugin_cpp_gen_aimrt_cpp_rpc/ProtobufAimRTRpcGenCode.cmake)'.format(code_site_root_path_url) }}: Generates C++ service code with one CMake method:
- `add_protobuf_aimrt_rpc_gencode_target_for_proto_files`: Generates C++ service code for specified `.proto` files with parameters:
  - **TARGET_NAME**: Generated CMake target name
  - **PROTO_FILES**: List of protocol file paths
  - **GENCODE_PATH**: Generated code output path
  - **DEP_PROTO_TARGETS**: Dependent Proto CMake targets
  - **OPTIONS**: Additional protoc arguments

These methods should be used sequentially: first generate message structure code, then service code. Example usage:
```cmake
# Generate C++ code for all '.proto' files in the current folder
add_protobuf_gencode_target_for_proto_path(
  TARGET_NAME example_pb_gencode
  PROTO_PATH ${CMAKE_CURRENT_SOURCE_DIR}
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR})
```

```markdown
## Generate RPC service C++ code for 'rpc.proto' file. Need to rely on 'example_pb_gencode'

```cmake
add_protobuf_aimrt_rpc_gencode_target_for_proto_files(
  TARGET_NAME example_rpc_aimrt_rpc_gencode
  PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/rpc.proto
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS example_pb_gencode)
```
```

之后只要链接`example_rpc_aimrt_rpc_gencode`这个 CMake Target 即可使用该协议。例如：
```cmake
target_link_libraries(my_lib PUBLIC example_rpc_aimrt_rpc_gencode)
```
```

### ROS2 Srv

ROS2 Srv 是一种用于在 ROS2 中进行 RPC 定义的格式。在使用时，开发者需要先定义一个 ROS2 Package，在其中定义一个`.srv`文件，比如`example.srv`：

```
byte[]  data
---
int64   code
```
```

其中，以`---`来分割 Req 和 Rsp 的定义。然后直接通过 ROS2 提供的 CMake 方法`rosidl_generate_interfaces`，为 Req 和 Rsp 消息生成 C++ 代码和 CMake Target，例如：
```cmake
rosidl_generate_interfaces(
  example_srv_gencode
  "srv/example.srv"
)
```
```

之后就可以引用相关的 CMake Target 来使用生成的 Req 和 Rsp 的消息结构 C++ 代码。详情请参考 ROS2 的官方文档和 AimRT 提供的 Example。

在生成了 Req 和 Rsp 消息结构的 C++ 代码后，开发者还需要使用 AimRT 提供的 Python 脚本工具，生成服务定义部分的 C++ 桩代码，例如：
```shell
python3 ARGS ./ros2_py_gen_aimrt_cpp_rpc.py --pkg_name=example_pkg --srv_file=./example.srv --output_path=./
```
```

这将生成`example.aimrt_rpc.srv.h`和`example.aimrt_rpc.srv.cc`文件，包含了根据定义的服务生成的 C++ 类和方法。


请注意，以上这套为 ROS2 生成 C++ 服务代码的过程只是为了给开发者展示底层的原理，实际使用时还需要手动处理依赖和 CMake 封装等方面的问题，比较繁琐。AimRT 对这个过程进行了一定的封装，开发者可以直接使用{{ '[Ros2AimRTRpcGenCode.cmake]({}/src/tools/ros2_py_gen_aimrt_cpp_rpc/Ros2AimRTRpcGenCode.cmake)'.format(code_site_root_path_url) }}文件中提供的 CMake 方法：

- `add_ros2_aimrt_rpc_gencode_target_for_one_file`：为单个 srv 文件生成 RPC 服务 C++ 代码，参数如下：
  - **TARGET_NAME**：生成的 CMake Target 名称；
  - **PACKAGE_NAME**：ROS2 协议 PKG 的名称；
  - **PROTO_FILE**：协议文件的路径；
  - **GENCODE_PATH**：生成的桩代码存放路径；
  - **DEP_PROTO_TARGETS**：依赖的协议 CMake Target；
  - **OPTIONS**：传递给工具的其他参数；


实际使用时，需要先生成消息结构体部分的 C++ 代码，再生成 C++ 服务代码，以下是一个示例：
```cmake
# Generate C++ code for Req and Rsp message in `.srv` file
rosidl_generate_interfaces(
  example_srv_gencode
  "srv/example.srv"
)

# Generate RPC service C++ code for the example '.srv' file. It is necessary to rely on the CMake Target related to ROS2 messages, which is defined in '${ROS2_EXAMPLE_CMAKE_TARGETS}'
add_ros2_aimrt_rpc_gencode_target_for_one_file(
  TARGET_NAME example_ros2_rpc_aimrt_rpc_gencode
  PACKAGE_NAME example_pkg
  PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/srv/example.srv
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS
    rclcpp::rclcpp
    ${ROS2_EXAMPLE_CMAKE_TARGETS})
```
```

之后只要链接`example_ros2_rpc_aimrt_rpc_gencode`这个 CMake Target 即可使用该协议。例如：
```cmake
target_link_libraries(my_lib PUBLIC example_ros2_rpc_aimrt_rpc_gencode)
```

## RpcHandle

In Agibot, modules can obtain the `aimrt::rpc::RpcHandleRef` handle by calling the `GetRpcHandle()` interface of the `CoreRef` handle. Typically, developers do not directly use the interfaces provided by `aimrt::rpc::RpcHandleRef`, but instead generate stub code based on RPC IDL files to encapsulate the `RpcHandleRef` handle, then use these encapsulated interfaces in business logic.

The specific forms of these encapsulated interfaces will be introduced in subsequent chapters of this document. Developers should follow these steps when using RPC functionality:
- Client side:
  - During `Initialize` phase, call interfaces for **registering RPC Client methods**;
  - During `Start` phase, call **RPC Invoke** interfaces to implement RPC calls;
- Server side:
  - During `Initialize` phase, call interfaces for **registering RPC Server services**;

Agibot officially supports two protocol IDLs: **Protobuf** and **Ros2 Srv**, providing code generation tools for both. The generated RPC interfaces maintain consistent API styles across different protocol types.

Developers can also use the `MergeServerContextToClientContext` method to propagate server-side context information to client-side context, which facilitates end-to-end data link tracing. For details, refer to the Context chapter documentation.
```

## Status

During RPC calls or RPC processing, users can obtain error status through a variable of type `aimrt::rpc::Status`, which contains the following interfaces:
```cpp
namespace aimrt::rpc {

class Status {
 public:
  explicit Status(aimrt_rpc_status_code_t code);
  explicit Status(uint32_t code);

  bool OK() const;
  operator bool() const;

  void SetCode(uint32_t code);
  void SetCode(aimrt_rpc_status_code_t code);

  uint32_t Code() const;

  std::string ToString() const;

  static std::string_view GetCodeMsg(uint32_t code);
};

}  // namespace aimrt::rpc
```

The `Status` type is very lightweight, containing only an error code field. Users can set this code through constructors or Set methods, and retrieve it through Get methods. The enumeration values of error codes can be found in {{ '[rpc_status_base.h]({}/src/interface/aimrt_module_c_interface/rpc/rpc_status_base.h)'.format(code_site_root_path_url) }} as follows:

| Error Type | Error Code | Value | Description |
|------|--------|----|------|
| General | `AIMRT_RPC_STATUS_OK` | 0 | Success |
| General | `AIMRT_RPC_STATUS_UNKNOWN` | 1 | Unknown error |
| General | `AIMRT_RPC_STATUS_TIMEOUT` | 2 | Timeout |
| Server | `AIMRT_RPC_STATUS_SVR_UNKNOWN` | 1000 | Server unknown error |
| Server | `AIMRT_RPC_STATUS_SVR_BACKEND_INTERNAL_ERROR` | 1001 | Server internal error |
| Server | `AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED` | 1002 | Service not implemented |
| Server | `AIMRT_RPC_STATUS_SVR_NOT_FOUND` | 1003 | Service not found |
| Server | `AIMRT_RPC_STATUS_SVR_INVALID_SERIALIZATION_TYPE` | 1004 | Invalid serialization type |
| Server | `AIMRT_RPC_STATUS_SVR_SERIALIZATION_FAILED` | 1005 | Serialization failed |
| Server | `AIMRT_RPC_STATUS_SVR_INVALID_DESERIALIZATION_TYPE` | 1006 | Invalid deserialization type |
| Server | `AIMRT_RPC_STATUS_SVR_DESERIALIZATION_FAILED` | 1007 | Deserialization failed |
| Server | `AIMRT_RPC_STATUS_SVR_HANDLE_FAILED` | 1008 | Processing failed |
| Client | `AIMRT_RPC_STATUS_CLI_UNKNOWN` | 2000 | Client unknown error |
| Client | `AIMRT_RPC_STATUS_CLI_FUNC_NOT_REGISTERED` | 2001 | Function not registered |
| Client | `AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR` | 2002 | Client internal error |
| Client | `AIMRT_RPC_STATUS_CLI_INVALID_CONTEXT` | 2003 | Invalid context |
| Client | `AIMRT_RPC_STATUS_CLI_INVALID_ADDR` | 2004 | Invalid address |
| Client | `AIMRT_RPC_STATUS_CLI_INVALID_SERIALIZATION_TYPE` | 2005 | Invalid serialization type |
| Client | `AIMRT_RPC_STATUS_CLI_SERIALIZATION_FAILED` | 2006 | Serialization failed |
| Client | `AIMRT_RPC_STATUS_CLI_INVALID_DESERIALIZATION_TYPE` | 2007 | Invalid deserialization type |
| Client | `AIMRT_RPC_STATUS_CLI_DESERIALIZATION_FAILED` | 2008 | Deserialization failed |
| Client | `AIMRT_RPC_STATUS_CLI_NO_BACKEND_TO_HANDLE` | 2009 | No backend available |
| Client | `AIMRT_RPC_STATUS_CLI_SEND_REQ_FAILED` | 2010 | Request sending failed |

Note that `Status` error messages generally indicate framework-level errors (e.g., service not found, network errors, or serialization errors) for developers to troubleshoot framework issues. For business-level errors, developers should add corresponding fields in business packages.

**Special Case for ROS2 RPC Backend with ROS2 Srv**: Due to ROS2's limitation of not supporting return fields other than request_id and response, the framework will return `AIMRT_RPC_STATUS_OK` even when server-side errors occur.
For example, when a service is not implemented (which should return `AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED`), the framework will still return `AIMRT_RPC_STATUS_OK` to clients under this specific combination.

## Client

In Agibot RPC stub code generation outputs (e.g., `rpc.aimrt_rpc.pb.h` or `example.aimrt_rpc.srv.h`), four types of Client Proxy interfaces are provided for initiating RPC calls:
- **Synchronous Interface**: Named `XXXSyncProxy`
- **Asynchronous Callback Interface**: Named `XXXAsyncProxy`
- **Asynchronous Future Interface**: Named `XXXFutureProxy`
- **Coroutine-based Interface**: Named `XXXCoProxy`

These Proxy types can be used interchangeably. Developers should choose appropriate types based on actual needs. They share identical underlying behavior despite differing in RPC invocation APIs.

### Public Interfaces

All Proxies share a common base class with public interfaces as follows:
```cpp
class ProxyBase {
 public:
  std::string_view RpcType() const;

  void SetServiceName(std::string_view service_name);
  std::string_view ServiceName() const;

  std::shared_ptr<Context> NewContextSharedPtr(ContextRef ctx_ref = ContextRef()) const;

  void SetDefaultContextSharedPtr(const std::shared_ptr<Context>& ctx_ptr);
  std::shared_ptr<Context> GetDefaultContextSharedPtr() const;
};

class XXXProxy : public aimrt::rpc::CoProxyBase {
 public:
  explicit XXXProxy(aimrt::rpc::RpcHandleRef rpc_handle_ref);

  static bool RegisterClientFunc(aimrt::rpc::RpcHandleRef rpc_handle_ref);
  static bool RegisterClientFunc(aimrt::rpc::RpcHandleRef rpc_handle_ref, std::string_view service_name);

  // ...
}
```

Key specifications:
- Proxies are generally lightweight and can be created on demand
- RPC Type is an inherent attribute indicating the protocol system (e.g. `pb`, `ros2`), accessible via `RpcType` method
- RPC ServiceName defaults to a protocol-bound value. Use `SetServiceName` to customize when needing different services with same protocol
- All Proxies must be constructed from `aimrt::rpc::RpcHandleRef` handle
- All Proxies provide `RegisterClientFunc` static method for RPC Client registration:
  - This actually calls a generated `RegisterXXXClientFunc` global method
  - Requires `aimrt::rpc::RpcHandleRef` handle as parameter
  - Optionally accepts ServiceName for registration
- Different Proxies of same type are distinguished by `ServiceName`. Developers must ensure registration/usage consistency
- Default Context configuration:
  - Uses default Context when no/empty Context provided in RPC calls
  - Configure via `SetDefaultContextSharedPtr`/`GetDefaultContextSharedPtr`
  - Clone new Context from default via `NewContextSharedPtr`


### Synchronous Interfaces

Synchronous interfaces offer simplest usage but lowest efficiency. They block current thread until RPC returns. Suitable for non-performance-critical scenarios to improve development efficiency, not recommended for high-performance requirements.

Usage steps:
- **Step 0**: Include stub header (e.g. `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`) containing `XXXSyncProxy` handle
- **Step 1**: Call `RegisterClientFunc` during `Initialize` phase
- **Step 2**: Initiate RPC call in business logic during `Start` phase:
  - **Step 2-1**: Create `XXXSyncProxy` with `aimrt::rpc::RpcHandleRef` handle
  - **Step 2-2**: Create and populate Req/Rsp objects
  - **Step 2-3**: [Optional] Create ctx with timeout settings
  - **Step 2-4**: Call RPC via proxy with ctx/Req/Rsp. Keep all parameters valid and unchanged during call
  - **Step 2-5**: Parse status and Rsp

Protobuf-based example (ROS2 Srv syntax is similar):
```cpp
#include "rpc.aimrt_rpc.pb.h"

bool HelloWorldModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  // Step 1: RegisterClientFunc
  aimrt::protocols::example::RegisterExampleServiceClientFunc(core_.GetRpcHandle());

  return true;
}

// Step 2: Call rpc
void HelloWorldModule::Foo() {
  // Step 2-1: Create a proxy
  ExampleServiceSyncProxy proxy(core_.GetRpcHandle());

  // Step 2-2: Create req and rsp
  ExampleReq req;
  ExampleRsp rsp;
  req.set_msg("hello world");

  // Step 2-3: Create context
  auto ctx = proxy.NewContextSharedPtr();
  ctx->SetTimeout(std::chrono::seconds(3));

  // Step 2-4: Call rpc
  auto status = proxy.ExampleFunc(ctx, req, rsp);

  // Step 2-5: Parse rsp
  if (status.OK()) {
    auto msg = rsp.msg();
    // ...
  } else {
    // ...
  }
}
```

More examples:
- {{ '[pb_rpc_sync_client]({}/src/examples/cpp/pb_rpc/module/normal_rpc_sync_client_module/normal_rpc_sync_client_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_sync_client]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_sync_client_module/normal_rpc_sync_client_module.cc)'.format(code_site_root_path_url) }}

### Asynchronous Callback-style Interface

The asynchronous callback-style interface uses callbacks to return asynchronous results, offering the best performance but the least developer-friendly experience, as it can easily lead to callback hell.

The general steps for initiating RPC calls using asynchronous callback-style interfaces are:
- **Step 0**: Include stub code header files, such as `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contain the asynchronous interface handle `XXXAsyncProxy`;
- **Step 1**: Call the `RegisterClientFunc` method during the `Initialize` phase to register the RPC Client;
- **Step 2**: Initiate RPC calls in a business function during the `Start` phase:
  - **Step 2-1**: Create a `XXXAsyncProxy` with constructor parameter `aimrt::rpc::RpcHandleRef`;
  - **Step 2-2**: Create Req and Rsp objects, then populate Req content;
  - **Step 2-3**: [Optional] Create ctx and configure timeout information;
  - **Step 2-4**: Use the proxy to initiate RPC calls by passing ctx, Req, Rsp and result callback, ensuring ctx/Req/Rsp remain valid and unmodified throughout the call lifecycle;
  - **Step 2-5**: Obtain returned status and parse status/Rsp in the callback function;

The initial steps are essentially the same as synchronous interfaces, with the key difference being that **Step 2-4** requires using asynchronous callbacks to obtain results. Below is a simple protobuf-based example (ROS2 Srv syntax is similar):

```cpp
#include "rpc.aimrt_rpc.pb.h"

bool HelloWorldModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  // Step 1: RegisterClientFunc
  aimrt::protocols::example::RegisterExampleServiceClientFunc(core_.GetRpcHandle());

  return true;
}

// Step 2: Call rpc
void HelloWorldModule::Foo() {
  // Step 2-1: Create a proxy
  ExampleServiceAsyncProxy proxy(core_.GetRpcHandle());

  // Step 2-2: Create req and rsp
  // To ensure that the lifecycle of req and rsp is longer than RPC calls, we should use smart pointers here
  auto req = std::make_shared<ExampleReq>();
  auto rsp = std::make_shared<ExampleRsp>();
  req->set_msg("hello world");

  // Step 2-3: Create context
  // To ensure that the lifecycle of context is longer than RPC calls, we should use smart pointers here
  auto ctx = proxy.NewContextSharedPtr();
  ctx->SetTimeout(std::chrono::seconds(3));

  // Step 2-4: Call rpc with callback
  proxy.GetBarData(
      ctx, *req, *rsp,
      [this, ctx, req, rsp](aimrt::rpc::Status status) {
        // Step 2-5: Parse rsp
        if (status.OK()) {
          auto msg = rsp->msg();
          // ...
        } else {
          // ...
        }
      });
}
```

For more examples, please refer to:
- {{ '[pb_rpc_async_client]({}/src/examples/cpp/pb_rpc/module/normal_rpc_async_client_module/normal_rpc_async_client_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_async_client]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_async_client_module/normal_rpc_async_client_module.cc)'.format(code_site_root_path_url) }}

### Asynchronous Future-style Interface

The asynchronous Future-style interface returns asynchronous results based on `std::future`. Developers can perform other tasks after initiating an RPC call, then blockingly obtain the result by calling the `std::future::get` method when needed. It balances performance and development friendliness to some extent, serving as a middle ground between synchronous and asynchronous callback styles.

The process of using asynchronous Future-style interfaces for RPC calls typically involves these steps:
- **Step 0**: Include stub code header files (e.g., `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`) containing the asynchronous interface handle `XXXFutureProxy`;
- **Step 1**: Register the RPC Client using `RegisterClientFunc` during the `Initialize` phase;
- **Step 2**: Initiate RPC calls within business functions during the `Start` phase:
  - **Step 2-1**: Create an `XXXFutureProxy` with constructor parameter `aimrt::rpc::RpcHandleRef`;
  - **Step 2-2**: Create Req and Rsp objects, then populate Req content;
  - **Step 2-3**: [Optional] Create ctx to configure timeout and other information;
  - **Step 2-4**: Initiate RPC call using proxy with ctx, Req, Rsp and result callback. Ensure ctx, Req, and Rsp remain valid and unmodified throughout the call lifecycle. Obtain a `std::future<Status>` handle;
  - **Step 2-5**: At a subsequent time, blockingly call the `get()` method of the `std::future<Status>` handle to retrieve status value, then parse status and Rsp;

Here's a simple protobuf-based example (ROS2 Srv syntax is similar):
```cpp
#include "rpc.aimrt_rpc.pb.h"

bool HelloWorldModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  // Step 1: RegisterClientFunc
  aimrt::protocols::example::RegisterExampleServiceClientFunc(core_.GetRpcHandle());

  return true;
}

// Step 2: Call rpc
void HelloWorldModule::Foo() {
  // Step 2-1: Create a proxy
  ExampleServiceFutureProxy proxy(core_.GetRpcHandle());

  // Step 2-2: Create req and rsp
  ExampleReq req;
  ExampleRsp rsp;
  req.set_msg("hello world");

  // Step 2-3: Create context
  auto ctx = proxy.NewContextSharedPtr();
  ctx->SetTimeout(std::chrono::seconds(3));

  // Step 2-4: Call rpc, return 'std::future<Status>'
  auto status_future = proxy.ExampleFunc(ctx, req, rsp);

  // ...

  // Step 2-5: Call 'get()' method of 'status_future', Parse rsp
  auto status = status_future.get();
  if (status.OK()) {
    auto msg = rsp.msg();
    // ...
  } else {
    // ...
  }
}
```

More examples available at:
- {{ '[pb_rpc_future_client]({}/src/examples/cpp/pb_rpc/module/normal_rpc_future_client_module/normal_rpc_future_client_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_future_client]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_future_client_module/normal_rpc_future_client_module.cc)'.format(code_site_root_path_url) }}

### Stackless Coroutine-style Interface

AimRT provides a stackless coroutine-style interface for RPC clients based on C++20 coroutines and the current implementation of the [C++ executors proposal](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p0443r14.html) using the [libunifex](https://github.com/facebookexperimental/libunifex) library. The stackless coroutine interface essentially encapsulates asynchronous callback-style interfaces, maintaining comparable performance while significantly improving developer friendliness.

The general steps for making RPC calls using the coroutine-style interface are:
- **Step 0**: Include the stub code header file (e.g., `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`), which contains the coroutine interface handle `XXXCoProxy`;
- **Step 1**: Register the RPC client by calling the `RegisterClientFunc` method during the `Initialize` phase;
- **Step 2**: Initiate the RPC call within a business coroutine during the `Start` phase:
  - **Step 2-1**: Create an `XXXCoProxy` instance with constructor parameter `aimrt::rpc::RpcHandleRef`;
  - **Step 2-2**: Create Req and Rsp objects, then populate the Req content;
  - **Step 2-3**: [Optional] Create a context (ctx) and configure timeout information;
  - **Step 2-4**: Initiate the RPC call using the proxy with ctx, Req, Rsp and result callback. Wait for completion within the coroutine, ensuring ctx/Req/Rsp remain valid and unchanged throughout the call. Obtain the returned status;
  - **Step 2-5**: Parse the status and Rsp;

The interface style is almost identical to synchronous interfaces but must be called within coroutines. Below is a simple protobuf-based example (ROS2 Srv syntax is similar):

```cpp
#include "rpc.aimrt_rpc.pb.h"

bool HelloWorldModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  // Step 1: RegisterClientFunc
  aimrt::protocols::example::RegisterExampleServiceClientFunc(core_.GetRpcHandle());

  return true;
}

// Step 2: Call rpc
co::Task<void> HelloWorldModule::Foo() {
  // Step 2-1: Create a proxy
  ExampleServiceCoProxy proxy(core_.GetRpcHandle());

  // Step 2-2: Create req and rsp
  ExampleReq req;
  ExampleRsp rsp;
  req.set_msg("hello world");

  // Step 2-3: Create context
  auto ctx = proxy.NewContextSharedPtr();
  ctx->SetTimeout(std::chrono::seconds(3));

  // Step 2-4: Call rpc
  auto status = co_await proxy.ExampleFunc(ctx, req, rsp);

  // Step 2-5: Parse rsp
  if (status.OK()) {
    auto msg = rsp.msg();
    // ...
  } else {
    // ...
  }
}
```

For more examples, refer to:
- {{ '[pb_rpc_co_client]({}/src/examples/cpp/pb_rpc/module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_co_client]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)'.format(code_site_root_path_url) }}


## Server

In AimRT RPC stub code generated files (e.g., `rpc.aimrt_rpc.pb.h` or `example.aimrt_rpc.srv.h`), three types of Service base classes are provided. Developers implement these virtual interfaces to provide actual RPC services:
- **Synchronous Interface**: Typically named `XXXSyncService`;
- **Asynchronous Callback Interface**: Typically named `XXXAsyncService`;
- **Stackless Coroutine Interface**: Typically named `XXXCoService`;

Within a single service, these types cannot be mixed - only one type can be selected based on developer requirements.


### Common Interfaces

All Services share common base interfaces:
```cpp
class ServiceBase {
 public:
  std::string_view RpcType() const;

  void SetServiceName(std::string_view service_name);
  std::string_view ServiceName() const;

  // ...
};

class XXXService : public aimrt::rpc::ServiceBase {
  // ...
}
```

Key specifications:
- Developers must inherit these Service base classes to implement business logic. Service instance lifecycle management is the developer's responsibility;
- RPC Type is an inherent attribute indicating the RPC service's protocol system (e.g., `pb`, `ros2`). Accessible via the `RpcType` method;
- RPC ServiceName represents the service name. By default, it uses a protocol-bound value. Use `SetServiceName` to configure different services using the same protocol;


Best practice: If callback tasks are lightweight (e.g., setting variables), handle them directly in the callback. For heavy tasks, schedule them to dedicated executors.

### Synchronous Interface

Synchronous interfaces are the simplest to use, but in many cases when business RPC handler functions need to make subsequent downstream requests involving asynchronous calls, they can only block and wait for the downstream calls to complete. This may lead to reduced operational efficiency. They are generally suitable for handling simple requests that don't require initiating other asynchronous calls.

The implementation of RPC services using synchronous interfaces typically involves the following steps:
- **Step 0**: Include stub code header files, such as `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contain the synchronous interface's Service base class `XXXSyncService`;
- **Step 1**: Developers implement an Impl class that inherits from `XXXSyncService` and implements its virtual interfaces;
  - **Step 1-1**: Parse Req and populate Rsp;
  - **Step 1-2**: Return `Status`;
- **Step 2**: Call `RegisterService` method of `RpcHandleRef` during the `Initialize` phase to register the RPC Service;

Here's a simple protobuf-based example (ROS2 Srv syntax is similar):
```cpp
#include "rpc.aimrt_rpc.pb.h"

// Step 1: Implement an Impl class that inherits 'XXXSyncService'
class ExampleServiceSyncServiceImpl : public ExampleServiceSyncService {
 public:
  aimrt::rpc::Status ExampleFunc(
      aimrt::rpc::ContextRef ctx, const ExampleReq& req, ExampleRsp& rsp) override {
    // Step 1-1: Parse req and set rsp
    rsp.set_msg("echo " + req.msg());

    // Step 1-2: Return status
    return aimrt::rpc::Status();
  }
};

bool HelloWorldModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  // Step 2: Register rpc service
  service_ptr_ = std::make_shared<ExampleServiceSyncServiceImpl>();

  core_.GetRpcHandle().RegisterService(service_ptr_.get());

  return true;
}
```

More examples can be found at:
- {{ '[pb_rpc_sync_service]({}/src/examples/cpp/pb_rpc/module/normal_rpc_sync_server_module/service.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_sync_service]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_sync_server_module/service.cc)'.format(code_site_root_path_url) }}

### Asynchronous Callback-style Interface

Asynchronous callback-style interfaces pass a callback to developers, who invoke this callback to deliver final processing results after completing RPC handling. This approach allows initiating other asynchronous calls within RPC processing. While typically offering better performance due to non-blocking operation, it often results in code that's harder to read and maintain.

Implementing RPC services using asynchronous callback-style interfaces typically involves these steps:
- **Step 0**: Include stub code header files, such as `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contain the asynchronous interface's Service base class `XXXAsyncService`;
- **Step 1**: Developers implement an Impl class that inherits from `XXXAsyncService` and implements its virtual interfaces;
  - **Step 1-1**: Parse Req and populate Rsp;
  - **Step 1-2**: Invoke callback to return the `Status`;
- **Step 2**: Call `RegisterService` method of `RpcHandleRef` during the `Initialize` phase to register the RPC Service;

Here's a simple protobuf-based example (ROS2 Srv syntax is similar):
```cpp
// Step 1: Implement an Impl class that inherits 'XXXAsyncService'
class ExampleServiceAsyncServiceImpl : public ExampleServiceAsyncService {
 public:
  void ExampleFunc(
      aimrt::rpc::ContextRef ctx, const ExampleReq& req, ExampleRsp& rsp,
      std::function<void(aimrt::rpc::Status)>&& callback) override {
    // Step 1-1: Parse req and set rsp
    rsp.set_msg("echo " + req.msg());

    // Step 1-2: Return status by callback
    callback(aimrt::rpc::Status());
  }
};

bool HelloWorldModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  // Step 2: Register rpc service
  service_ptr_ = std::make_shared<ExampleServiceAsyncServiceImpl>();

  core_.GetRpcHandle().RegisterService(service_ptr_.get());

  return true;
}
```

More examples can be found at:
- {{ '[pb_rpc_async_service]({}/src/examples/cpp/pb_rpc/module/normal_rpc_async_server_module/service.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_async_service]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_async_server_module/service.cc)'.format(code_site_root_path_url) }}

### Stackless Coroutine Interface

Similar to the RPC client side, AimRT also provides a stackless coroutine-based interface on the RPC service side, implemented using C++20 coroutines and the current implementation library [libunifex](https://github.com/facebookexperimental/libunifex) of the [C++ executors proposal](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p0443r14.html). The stackless coroutine interface essentially encapsulates asynchronous callback-style interfaces, maintaining comparable performance while significantly improving developer friendliness.

Implementing RPC services using coroutine interfaces typically involves the following steps:
- **Step 0**: Include stub code header files (e.g., `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`) containing the coroutine interface base class `XXXCoService`;
- **Step 1**: Developers implement an Impl class inheriting from `XXXCoService` and override virtual interfaces:
  - **Step 1-1**: Parse requests (Req) and populate responses (Rsp);
  - **Step 1-2**: Return `Status` using co_return;
- **Step 2**: Register the RPC service by calling the `RegisterService` method of `RpcHandleRef` during the `Initialize` phase;

The interface style remains almost identical to synchronous interfaces. Here's a simple protobuf-based example (ROS2 Srv syntax is similar):

```cpp
// Step 1: Implement an Impl class that inherits 'XXXCoService'
class ExampleServiceCoServiceImpl : public ExampleServiceCoService {
 public:
  co::Task<aimrt::rpc::Status> ExampleFunc(
      aimrt::rpc::ContextRef ctx, const ExampleReq& req, ExampleRsp& rsp) override {
    // Step 1-1: Parse req and set rsp
    rsp.set_msg("echo " + req.msg());

    // Step 1-2: Return status by co_return
    co_return aimrt::rpc::Status();
  }
};

bool HelloWorldModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  // Step 2: Register rpc service
  service_ptr_ = std::make_shared<ExampleServiceCoServiceImpl>();

  core_.GetRpcHandle().RegisterService(service_ptr_.get());

  return true;
}
```

More examples can be found at:
- {{ '[pb_rpc_co_service]({}/src/examples/cpp/pb_rpc/module/normal_rpc_co_server_module/service.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_co_service]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_co_server_module/service.cc)'.format(code_site_root_path_url) }}

# Context

When invoking RPCs, developers can pass an `aimrt::rpc::Context`. When processing RPCs, they will receive an `aimrt::rpc::ContextRef`. The `ContextRef` type is a reference to the `Context` type, both containing essentially the same interfaces. Their primary functionality is to carry timeout configurations and key-value data for passing specific information to downstream services or RPC backends.

The interfaces are as follows:

```cpp
namespace aimrt::rpc {

class Context {
 public:
  bool CheckUsed() const;
  void SetUsed();
  void Reset();

  aimrt_rpc_context_type_t GetType() const;

  std::chrono::nanoseconds Timeout() const;
  void SetTimeout(std::chrono::nanoseconds timeout);

  std::string_view GetMetaValue(std::string_view key) const;
  void SetMetaValue(std::string_view key, std::string_view val);
  std::vector<std::string_view> GetMetaKeys() const;

  std::string ToString() const;
};

class ContextRef {
 public:
  ContextRef(const Context& ctx);
  ContextRef(const Context* ctx_ptr);
  ContextRef(const std::shared_ptr<Context>& ctx_ptr);
  explicit ContextRef(const aimrt_rpc_context_base_t* base_ptr);

  bool CheckUsed() const;
  void SetUsed();
  void Reset();

  aimrt_rpc_context_type_t GetType() const;

  std::chrono::nanoseconds Timeout() const;
  void SetTimeout(std::chrono::nanoseconds timeout);

  std::string_view GetMetaValue(std::string_view key) const;
  void SetMetaValue(std::string_view key, std::string_view val);
  std::vector<std::string_view> GetMetaKeys() const;

  std::string ToString() const;
};

}  // namespace aimrt::rpc
```

When using RPC ctx of type `Context` or `ContextRef`, note:
- RPC ctx has two types: Client-side and Server-side, determined during construction and unmodifiable. They are used in Client and Server scenarios respectively
- Use `SetTimeout` and `Timeout` methods to set/get timeout configurations in ctx
- Use `SetMetaValue` and `GetMetaValue` to set/get key-value pairs in ctx. Use `GetMetaKeys` to retrieve all current keys

AimRT defines some special keys in {{ '[rpc_context_base.h]({}/src/interface/aimrt_module_c_interface/rpc/rpc_context_base.h)'.format(code_site_root_path_url) }}. Follow these rules when using them:
- **AIMRT_RPC_CONTEXT_KEY_TO_ADDR**: Sets RPC peer address using standard URL format: `{{protocol}}://{{path}}`. The `{{protocol}}` field determines RPC backend selection, while `{{path}}` enables backend-specific customization
- **AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE**: Specifies message serialization type, must match registered type support
- **AIMRT_RPC_CONTEXT_KEY_FUNCTION_NAME**: Carries RPC function name
- **AIMRT_RPC_CONTEXT_KEY_BACKEND**: Passes actual backend name to Server-side

## Client-side Usage
`Context` primarily conveys special information to AimRT framework and RPC backends during RPC invocation. Key considerations:
- Developers can directly construct `Context` instances and manage their lifecycle
- Only Client-type ctx can be passed to Client-side RPC invocation methods
- Each `Context` can only be used for one Client-side invocation. After passing to RPC methods, its state becomes `Used`. Reusing without `Reset` will cause errors
- Client-side RPC methods actually accept `ContextRef` parameters, but `Context` can implicitly convert to `ContextRef`
- Timeout settings in ctx are backend-dependent. Refer to specific backend documentation
- Backends may process ctx metadata differently: some read specific keys, others transparently transmit all key-values. Consult backend documentation

## Server-side Usage
In callback handlers receiving `ContextRef` parameters:
- Framework manages ctx lifecycle, aligned with Req/Rsp lifecycle
- Server-type ctx passed to handlers are in `Used` state
- Received ctx may contain timeout and metadata determined by RPC backend. Refer to backend documentation

## Context Propagation
In complex systems where servers make downstream requests forming logical chains, two methods synchronize Server-side ctx to Client-side ctx:

1. Use `RpcHandleRef`'s `MergeServerContextToClientContext`:
```cpp
aimrt::rpc::RpcHandleRef rpc_handle;
ExampleServiceSyncProxy proxy;

// RPC server handle function
aimrt::rpc::Status ExampleServiceSyncServiceImpl::GetBarData(
    aimrt::rpc::ContextRef server_ctx, const GetBarDataReq& bar_req, GetBarDataRsp& bar_rsp) {
  GetFooDataReq foo_req;
  GetFooDataRsp foo_rsp;

  aimrt::rpc::Context client_ctx;
  rpc_handle.MergeServerContextToClientContext(server_ctx, client_ctx);

  auto foo_status = proxy.GetFooData(client_ctx, req, rsp);

  // ...

  return aimrt::rpc::Status();
}
```

2. Use Proxy's `NewContextSharedPtr` with Server-side ctx:
```cpp
ExampleServiceSyncProxy proxy;

// RPC server handle function
aimrt::rpc::Status ExampleServiceSyncServiceImpl::GetBarData(
    aimrt::rpc::ContextRef server_ctx, const GetBarDataReq& bar_req, GetBarDataRsp& bar_rsp) {
  GetFooDataReq foo_req;
  GetFooDataRsp foo_rsp;

  auto client_ctx = proxy.NewContextSharedPtr(server_ctx);

  auto foo_status = proxy.GetFooData(client_ctx, req, rsp);

  // ...

  return aimrt::rpc::Status();
}
```




