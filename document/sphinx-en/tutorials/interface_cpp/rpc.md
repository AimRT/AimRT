# Rpc

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

Protocols are used to define the message format between RPC clients and servers. Generally, protocols are described using an IDL (Interface Description Language) that is programming language-independent, and then converted into code for various languages using specific tools. For RPC, this process requires two steps:
- As introduced in the [Channel](./channel.md) section, developers first need to use official tools to generate code for the **message types** defined in the protocol files for the target programming language;
- Developers then need to use tools provided by AimRT to generate code for the **service definitions** in the protocol files for the specified programming language;

### Protobuf

[Protobuf](https://protobuf.dev/) is a lightweight, efficient data interchange format developed by Google for serializing structured data. It is a widely used IDL that can describe message structures and also provides the `service` statement to define RPC services.

When using it, developers first need to define a `.proto` file containing message structures and RPC services. For example, `rpc.proto`:

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

Then, use the protoc tool provided by Protobuf to generate C++ code for the message structure part. For example:
```shell
protoc --cpp_out=. rpc.proto
```

This will generate `rpc.pb.h` and `rpc.pb.cc` files, containing C++ classes and methods for the defined message types.

After that, developers need to use the protoc plugin provided by AimRT to generate C++ code for the service definition part. For example:
```shell
protoc --aimrt_rpc_out=. --plugin=protoc-gen-aimrt_rpc=./protoc_plugin_py_gen_aimrt_cpp_rpc.py rpc.proto
```

This will generate `rpc.aimrt_rpc.pb.h` and `rpc.aimrt_rpc.pb.cc` files, containing C++ classes and methods for the defined services.

Note that this native code generation approach is mainly to demonstrate the underlying principles to developers. In practice, manual handling of dependencies and CMake packaging can be cumbersome. AimRT has encapsulated this process to some extent, allowing developers to directly use the CMake methods included in the following two files:

1. {{ '[ProtobufGenCode.cmake]({}/cmake/ProtobufGenCode.cmake)'.format(code_site_root_path_url) }}: Used to generate C++ code for the message structure part, containing two CMake methods:
- `add_protobuf_gencode_target_for_proto_path`: Generates C++ code for `.proto` files in a specified path, with the following parameters:
  - **TARGET_NAME**: The name of the generated CMake Target;
  - **PROTO_PATH**: The directory containing the protocol files;
  - **GENCODE_PATH**: The output path for the generated stub code;
  - **DEP_PROTO_TARGETS**: Dependent Proto CMake Targets;
  - **OPTIONS**: Additional arguments passed to protoc;
- `add_protobuf_gencode_target_for_one_proto_file`: Generates C++ code for a single `.proto` file, with the following parameters:
  - **TARGET_NAME**: The name of the generated CMake Target;
  - **PROTO_FILE**: The path to a single protocol file;
  - **GENCODE_PATH**: The output path for the generated stub code;
  - **DEP_PROTO_TARGETS**: Dependent Proto CMake Targets;
  - **OPTIONS**: Additional arguments passed to protoc;

2. {{ '[ProtobufAimRTRpcGenCode.cmake]({}/src/tools/protoc_plugin_cpp_gen_aimrt_cpp_rpc/ProtobufAimRTRpcGenCode.cmake)'.format(code_site_root_path_url) }}: Used to generate C++ service code, containing one CMake method:
- `add_protobuf_aimrt_rpc_gencode_target_for_proto_files`: Generates C++ service code for specified `.proto` files, with the following parameters:
  - **TARGET_NAME**: The name of the generated CMake Target;
  - **PROTO_FILES**: Paths to the protocol files;
  - **GENCODE_PATH**: The output path for the generated stub code;
  - **DEP_PROTO_TARGETS**: Dependent Proto CMake Targets;
  - **OPTIONS**: Additional arguments passed to protoc;

These methods should be used together: first generate C++ code for the message structures, then generate C++ service code. Example usage:
```cmake
# Generate C++ code for all '.proto' files in the current folder
add_protobuf_gencode_target_for_proto_path(
  TARGET_NAME example_pb_gencode
  PROTO_PATH ${CMAKE_CURRENT_SOURCE_DIR}
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR})
```
Generate RPC service C++ code for 'rpc.proto' file. Need to rely on 'example_pb_gencode':
```cmake
add_protobuf_aimrt_rpc_gencode_target_for_proto_files(
  TARGET_NAME example_rpc_aimrt_rpc_gencode
  PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/rpc.proto
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS example_pb_gencode)
```

After that, just link the `example_rpc_aimrt_rpc_gencode` CMake Target to use the protocol. For example:
```cmake
target_link_libraries(my_lib PUBLIC example_rpc_aimrt_rpc_gencode)
```

### ROS2 Srv

ROS2 Srv is a format used for RPC definition in ROS2. When using it, developers first need to define a ROS2 Package, and then define a `.srv` file in it, for example, `example.srv`:

```
byte[]  data
---
int64   code
```

Here, the definition of Req and Rsp is separated by `---`. Then, directly use the CMake method `rosidl_generate_interfaces` provided by ROS2 to generate C++ code and CMake Target for Req and Rsp messages, for example:
```cmake
rosidl_generate_interfaces(
  example_srv_gencode
  "srv/example.srv"
)
```

After that, you can reference the relevant CMake Target to use the generated C++ code for Req and Rsp message structures. For details, please refer to the official ROS2 documentation and the Example provided by AimRT.

After generating the C++ code for Req and Rsp message structures, developers also need to use the Python script tool provided by AimRT to generate the C++ stub code for the service definition part, for example:

## RpcHandle

In AimRT, modules can obtain the `aimrt::rpc::RpcHandleRef` handle by calling the `GetRpcHandle()` interface of the `CoreRef` handle. Generally, developers will not directly use the interfaces provided by `aimrt::rpc::RpcHandleRef`. Instead, they will generate some stub code based on the RPC IDL files to encapsulate the `RpcHandleRef` handle, and then use these encapsulated interfaces in business code.

The specific forms of these encapsulated interfaces will be introduced in subsequent sections of this document. When using RPC functionality, developers need to follow these steps to use these interfaces:
- Client side:
  - During the `Initialize` phase, call the **RPC Client method registration** interface;
  - During the `Start` phase, call the **RPC Invoke** interface to implement RPC calls;
- Server side:
  - During the `Initialize` phase, call the **RPC Server service registration** interface;

AimRT officially supports two protocol IDLs: **Protobuf** and **Ros2 Srv**, and provides tools for generating stub code for these two protocol IDLs. The generated RPC interfaces have the same API style except for the protocol type differences.

Developers can also use the `MergeServerContextToClientContext` method to pass context information from the server side to the client side, which can be used to connect the entire data link. For details, please refer to the Context chapter.## Status

During RPC calls or RPC processing, users can obtain error conditions in the RPC process through a variable of type `aimrt::rpc::Status`. Its included interfaces are as follows:
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

The `Status` type is very lightweight, containing only an error code field. Users can set this Code via constructors or Set methods, and retrieve it via Get methods. The enumeration values of error codes can be found in the {{ '[rpc_status_base.h]({}/src/interface/aimrt_module_c_interface/rpc/rpc_status_base.h)'.format(code_site_root_path_url) }} file, listed below:

| Error Code Type | Error Code | Value | Description |
|------|--------|----|------|
| General | `AIMRT_RPC_STATUS_OK` | 0 | Operation succeeded |
| General | `AIMRT_RPC_STATUS_UNKNOWN` | 1 | Unknown error |
| General | `AIMRT_RPC_STATUS_TIMEOUT` | 2 | Timeout |
| Server | `AIMRT_RPC_STATUS_SVR_UNKNOWN` | 1000 | Server-side unknown error |
| Server | `AIMRT_RPC_STATUS_SVR_BACKEND_INTERNAL_ERROR` | 1001 | Server-side internal error |
| Server | `AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED` | 1002 | Service not implemented |
| Server | `AIMRT_RPC_STATUS_SVR_NOT_FOUND` | 1003 | Service not found |
| Server | `AIMRT_RPC_STATUS_SVR_INVALID_SERIALIZATION_TYPE` | 1004 | Invalid serialization type |
| Server | `AIMRT_RPC_STATUS_SVR_SERIALIZATION_FAILED` | 1005 | Serialization failed |
| Server | `AIMRT_RPC_STATUS_SVR_INVALID_DESERIALIZATION_TYPE` | 1006 | Invalid deserialization type |
| Server | `AIMRT_RPC_STATUS_SVR_DESERIALIZATION_FAILED` | 1007 | Deserialization failed |
| Server | `AIMRT_RPC_STATUS_SVR_HANDLE_FAILED` | 1008 | Processing failed |
| Client | `AIMRT_RPC_STATUS_CLI_UNKNOWN` | 2000 | Client-side unknown error |
| Client | `AIMRT_RPC_STATUS_CLI_FUNC_NOT_REGISTERED` | 2001 | Function not registered |
| Client | `AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR` | 2002 | Client-side internal error |
| Client | `AIMRT_RPC_STATUS_CLI_INVALID_CONTEXT` | 2003 | Invalid context |
| Client | `AIMRT_RPC_STATUS_CLI_INVALID_ADDR` | 2004 | Invalid address |
| Client | `AIMRT_RPC_STATUS_CLI_INVALID_SERIALIZATION_TYPE` | 2005 | Invalid serialization type |
| Client | `AIMRT_RPC_STATUS_CLI_SERIALIZATION_FAILED` | 2006 | Serialization failed |
| Client | `AIMRT_RPC_STATUS_CLI_INVALID_DESERIALIZATION_TYPE` | 2007 | Invalid deserialization type |
| Client | `AIMRT_RPC_STATUS_CLI_DESERIALIZATION_FAILED` | 2008 | Deserialization failed |
| Client | `AIMRT_RPC_STATUS_CLI_NO_BACKEND_TO_HANDLE` | 2009 | No backend available for processing |
| Client | `AIMRT_RPC_STATUS_CLI_SEND_REQ_FAILED` | 2010 | Request sending failed |

Please note that the error information in `Status` generally only indicates framework-level errors, such as service not found, network errors, or serialization errors, to help developers troubleshoot framework-level issues. If developers need to return business-level errors, it is recommended to add corresponding fields in the business package.

Additionally, when using the **ROS2 RPC backend combined with ROS2 Srv**, since ROS2 itself does not support returning fields other than request_id and response, the framework side will not return the error codes provided by the server but will directly return an `AIMRT_RPC_STATUS_OK`.
For example, if a service on the server side is not implemented, it should return an `AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED` error code. However, due to the inherent limitations of this combination, the framework side will only return `AIMRT_RPC_STATUS_OK` to the client.

## Client

In the code generated by the AimRT RPC stub code tool, such as `rpc.aimrt_rpc.pb.h` or `example.aimrt_rpc.srv.h` files, four types of Client Proxy interfaces are provided. Developers use these Proxy interfaces to initiate RPC calls:
- **Synchronous Interface**: Generally named `XXXSyncProxy`;
- **Asynchronous Callback Interface**: Generally named `XXXAsyncProxy`;
- **Asynchronous Future Interface**: Generally named `XXXFutureProxy`;
- **Stackless Coroutine Interface**: Generally named `XXXCoProxy`;

These Proxy types can be used in combination. Developers can choose the appropriate type based on actual needs. Apart from differences in API interfaces when calling RPC, their underlying runtime behavior is consistent.### Public Interfaces

All Proxies share a common base class with some public interfaces, as shown below:
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

Detailed explanations are as follows:
- Proxies are generally lightweight and can be created on demand;
- RPC Type is an inherent property indicating the system to which the RPC Service name belongs, such as `pb`, `ros2`, etc. It can be obtained via the `RpcType` method;
- RPC ServiceName represents the name of the RPC service. If not specially configured, it will use a default value bound to the **protocol name**. If you need to provide different services using the same protocol, you can also set it via the `SetServiceName` method;
- All types of Proxies need to be constructed from the `aimrt::rpc::RpcHandleRef` handle;
- All types of Proxies provide the `RegisterClientFunc` static method for registering the current RPC Client:
  - This method actually calls a globally generated `RegisterXXXClientFunc` method for the current RPC;
  - This method requires passing an `aimrt::rpc::RpcHandleRef` handle as a parameter;
  - This method can optionally pass an RPC ServiceName field as the RPC service name during registration;
- If there are multiple Proxies of the same type, they are distinguished by `ServiceName`. Developers must ensure that the `ServiceName` used during registration and usage is consistent;
- A default Context can be set for the Proxy:
  - If no Context is passed or an empty Context is passed during an RPC call, the Proxy's default Context will be used;
  - Users can set or retrieve the default Context via the `SetDefaultContextSharedPtr` and `GetDefaultContextSharedPtr` methods;
  - Users can obtain a new Context copied from the default Context via the `NewContextSharedPtr` method;



### Synchronous Interfaces

Synchronous interfaces are the simplest to use but have the lowest runtime efficiency. They block the current thread while waiting for the RPC interface to return. This approach can be used in scenarios where performance is not critical to improve development efficiency, but it is not recommended for high-performance scenarios.


Using synchronous interfaces to initiate RPC calls is very straightforward and generally involves the following steps:
- **Step 0**: Include the stub code header file, such as `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contains the synchronous interface handle `XXXSyncProxy`;
- **Step 1**: Call the `RegisterClientFunc` method during the `Initialize` phase to register the RPC Client;
- **Step 2**: Initiate the RPC call in a business function during the `Start` phase:
  - **Step 2-1**: Create an `XXXSyncProxy` with the constructor parameter being an `aimrt::rpc::RpcHandleRef` handle;
  - **Step 2-2**: Create Req and Rsp, and populate the Req content;
  - **Step 2-3**: [Optional] Create a ctx and set timeout or other information;
  - **Step 2-4**: Use the proxy to pass ctx, Req, and Rsp to initiate the RPC call, synchronously waiting for the RPC call to complete. Ensure that ctx, Req, and Rsp remain valid and unmodified throughout the call cycle, and finally obtain the returned status;
  - **Step 2-5**: Parse the status and Rsp;


Below is a simple example based on protobuf. The syntax for ROS2 Srv is similar:
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


For more examples, please refer to:
- {{ '[pb_rpc_sync_client]({}/src/examples/cpp/pb_rpc/module/normal_rpc_sync_client_module/normal_rpc_sync_client_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_sync_client]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_sync_client_module/normal_rpc_sync_client_module.cc)'.format(code_site_root_path_url) }}### Asynchronous Callback-based Interfaces

Asynchronous callback-based interfaces use callbacks to return asynchronous results, offering the best performance but the lowest developer-friendliness, as they can easily lead to callback hell.

The general steps for initiating an RPC call using an asynchronous callback-based interface are as follows:
- **Step 0**: Include the stub code header file, such as `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contains the asynchronous interface handle `XXXAsyncProxy`;
- **Step 1**: Register the RPC Client by calling the `RegisterClientFunc` method during the `Initialize` phase;
- **Step 2**: Initiate the RPC call in a business function during the `Start` phase:
  - **Step 2-1**: Create an `XXXAsyncProxy`, with the constructor parameter being `aimrt::rpc::RpcHandleRef`;
  - **Step 2-2**: Create Req and Rsp, and populate the Req content;
  - **Step 2-3**: **[Optional]** Create ctx and set timeout and other information;
  - **Step 2-4**: Use the proxy to initiate the RPC call by passing ctx, Req, Rsp, and the result callback, ensuring that ctx, Req, and Rsp remain valid and unmodified throughout the call cycle;
  - **Step 2-5**: In the callback function, retrieve the returned status and parse the status and Rsp;

The initial steps are largely the same as for synchronous interfaces, with the key difference being that **Step 2-4** requires using an asynchronous callback to obtain the result. Below is a simple example based on protobuf; the syntax for ROS2 Srv is very similar:
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
- {{ '[ros2_rpc_async_client]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_async_client_module/normal_rpc_async_client_module.cc)'.format(code_site_root_path_url) }}### Asynchronous Future-style Interface

The asynchronous Future-style interface is based on `std::future` to return asynchronous results. Developers can initiate an RPC call and then proceed with other tasks, later blocking to retrieve the RPC result by calling the `std::future::get` method when needed. It strikes a balance between performance and developer-friendliness, serving as a middle ground between synchronous and asynchronous callback-style approaches.

Using the asynchronous Future-style interface to initiate an RPC call generally involves the following steps:
- **Step 0**: Include the stub code header file, such as `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contains the asynchronous interface handle `XXXFutureProxy`;
- **Step 1**: Register the RPC Client by calling the `RegisterClientFunc` method during the `Initialize` phase;
- **Step 2**: Initiate the RPC call in a business function during the `Start` phase:
  - **Step 2-1**: Create an `XXXFutureProxy` with the constructor parameter `aimrt::rpc::RpcHandleRef`;
  - **Step 2-2**: Create Req and Rsp objects, and populate the Req content;
  - **Step 2-3**: **[Optional]** Create a ctx and set timeout or other information;
  - **Step 2-4**: Use the proxy to initiate the RPC call by passing ctx, Req, Rsp, and a result callback. Ensure that ctx, Req, and Rsp remain valid and unmodified throughout the call cycle, and obtain a `std::future<Status>` handle;
  - **Step 2-5**: At a later time, blockingly call the `get()` method of the `std::future<Status>` handle to retrieve the status value, then parse the status and Rsp;

Here is a simple protobuf-based example (the syntax for ROS2 Srv is largely similar):
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

For more examples, refer to:
- {{ '[pb_rpc_future_client]({}/src/examples/cpp/pb_rpc/module/normal_rpc_future_client_module/normal_rpc_future_client_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_future_client]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_future_client_module/normal_rpc_future_client_module.cc)'.format(code_site_root_path_url) }}### Stackless Coroutine-based Interface

AimRT provides a set of stackless coroutine-style interfaces for RPC clients, implemented based on C++20 coroutines and the current implementation library [libunifex](https://github.com/facebookexperimental/libunifex) of the [C++ executors proposal](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p0443r14.html). The stackless coroutine interface essentially encapsulates asynchronous callback-style interfaces, offering comparable performance while significantly improving developer friendliness.

Using coroutine-style interfaces to initiate RPC calls generally involves the following steps:
- **Step 0**: Include the stub code header file, such as `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contains the coroutine interface handle `XXXCoProxy`;
- **Step 1**: Call the `RegisterClientFunc` method during the `Initialize` phase to register the RPC client;
- **Step 2**: Initiate the RPC call within a business coroutine during the `Start` phase:
  - **Step 2-1**: Create an `XXXCoProxy`, with the constructor parameter being `aimrt::rpc::RpcHandleRef`;
  - **Step 2-2**: Create Req and Rsp, and populate the Req content;
  - **Step 2-3**: **[Optional]** Create a ctx and set timeout or other information;
  - **Step 2-4**: Based on the proxy, pass in ctx, Req, Rsp, and the result callback to initiate the RPC call. Wait for the RPC call to complete within the coroutine, ensuring that ctx, Req, and Rsp remain valid and unmodified throughout the call cycle, and retrieve the returned status;
  - **Step 2-5**: Parse the status and Rsp;

The interface style is almost identical to synchronous interfaces but must be called within a coroutine. Below is a simple example based on protobuf (the syntax for ROS2 Srv is similar):
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

In the code generated by the AimRT RPC stub tool, such as `rpc.aimrt_rpc.pb.h` or `example.aimrt_rpc.srv.h`, three types of Service base classes are provided. Developers can inherit these base classes and implement their virtual interfaces to provide actual RPC services:
- **Synchronous Interface**: Typically named `XXXSyncService`;
- **Asynchronous Callback Interface**: Typically named `XXXAsyncService`;
- **Stackless Coroutine Interface**: Typically named `XXXCoService`;

Within a single service, these three types cannot be mixed; only one can be chosen. Developers can select the appropriate type based on their needs.


### Common Interfaces

All Services share a common base class with some shared interfaces, as shown below:
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

Specific explanations:
- Developers need to inherit these Service base classes to implement business logic. They are responsible for managing the lifecycle of business Service instances;
- RPC Type is an inherent attribute, indicating the system to which the RPC service name belongs, such as `pb`, `ros2`, etc. It can be retrieved via the `RpcType` method;
- RPC ServiceName represents the name of the RPC service. If not specially configured, it will use a default value bound to the **protocol name**. If the same protocol is used to provide different services, it can also be set via the `SetServiceName` method;



Best practice: If the task in the callback is very lightweight (e.g., setting a variable), it can be handled directly in the callback. However, if the task is heavy, it is better to schedule it to another dedicated task executor for processing.### Synchronous Interface

The synchronous interface is the simplest to use, but in many cases, the business RPC processing function needs to continue requesting downstream services, which involves some asynchronous calls. In such scenarios, it can only block and wait for the downstream calls to complete, potentially reducing runtime efficiency. Generally, synchronous interfaces are suitable for handling simple requests that do not require initiating other asynchronous calls.

To implement an RPC service using the synchronous interface, the following steps are typically followed:
- **Step 0**: Include the stub code header file, such as `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contains the synchronous interface's Service base class `XXXSyncService`;
- **Step 1**: The developer implements an Impl class that inherits `XXXSyncService` and overrides its virtual methods;
  - **Step 1-1**: Parse the Req and populate the Rsp;
  - **Step 1-2**: Return a `Status`;
- **Step 2**: During the `Initialize` phase, call the `RegisterService` method of `RpcHandleRef` to register the RPC Service;

Here is a simple example based on protobuf (the syntax for ROS2 Srv is similar):
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

For more examples, refer to:
- {{ '[pb_rpc_sync_service]({}/src/examples/cpp/pb_rpc/module/normal_rpc_sync_server_module/service.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_sync_service]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_sync_server_module/service.cc)'.format(code_site_root_path_url) }}

### Asynchronous Callback Interface

The asynchronous callback interface passes a callback to the developer, who invokes this callback to deliver the final processing result after the RPC processing is complete. This approach allows initiating other asynchronous calls within the RPC without blocking, typically yielding the best performance. However, it often results in code that is difficult to read and maintain.

To implement an RPC service using the asynchronous callback interface, the following steps are typically followed:
- **Step 0**: Include the stub code header file, such as `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contains the asynchronous interface's Service base class `XXXAsyncService`;
- **Step 1**: The developer implements an Impl class that inherits `XXXAsyncService` and overrides its virtual methods;
  - **Step 1-1**: Parse the Req and populate the Rsp;
  - **Step 1-2**: Invoke the callback to return the `Status`;
- **Step 2**: During the `Initialize` phase, call the `RegisterService` method of `RpcHandleRef` to register the RPC Service;

Here is a simple example based on protobuf (the syntax for ROS2 Srv is similar):
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

For more examples, refer to:
- {{ '[pb_rpc_async_service]({}/src/examples/cpp/pb_rpc/module/normal_rpc_async_server_module/service.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_async_service]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_async_server_module/service.cc)'.format(code_site_root_path_url) }}### Stackless Coroutine-Based Interface


Similar to the RPC Client side, AimRT also provides a set of stackless coroutine-based interfaces on the RPC Service side, implemented using C++20 coroutines and the current implementation library [libunifex](https://github.com/facebookexperimental/libunifex) from the [C++ executors proposal](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p0443r14.html). The stackless coroutine interface essentially encapsulates asynchronous callback-based interfaces, offering nearly identical performance while significantly improving developer friendliness.


Implementing an RPC service using coroutine-based interfaces generally involves the following steps:
- **Step 0**: Include the stub code header file, such as `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contains the coroutine-based Service base class `XXXCoService`;
- **Step 1**: The developer implements an Impl class that inherits from `XXXCoService` and overrides its virtual interfaces;
  - **Step 1-1**: Parse the Req and populate the Rsp;
  - **Step 1-2**: Use `co_return` to return a `Status`;
- **Step 2**: During the `Initialize` phase, register the RPC Service by calling the `RegisterService` method of `RpcHandleRef`;


The overall interface style is almost identical to synchronous interfaces. Below is a simple example based on protobuf, while the syntax for ROS2 Srv is largely similar:
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


For more examples, please refer to:
- {{ '[pb_rpc_co_service]({}/src/examples/cpp/pb_rpc/module/normal_rpc_co_server_module/service.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_co_service]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_co_server_module/service.cc)'.format(code_site_root_path_url) }}## Context

When developers make RPC calls, they can pass in an `aimrt::rpc::Context`. When processing RPC, they will also receive an `aimrt::rpc::ContextRef`. The `ContextRef` type is a reference to the `Context` type, and their interfaces are largely consistent. Their primary functions are to carry Timeout configurations and Key-Value data, used to pass specific information downstream or to the RPC backend.

The interface is as follows:

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

When using RPC ctx of type `Context` or `ContextRef`, note the following:
- RPC ctx is divided into Client-side and Server-side types, determined during construction and cannot be modified, used for Client and Server scenarios respectively;
- The `SetTimeout` and `Timeout` methods can be used to set and retrieve the timeout configuration in the ctx;
- The `SetMetaValue` and `GetMetaValue` methods can be used to set and retrieve Key-Value pairs in the ctx, and `GetMetaKeys` can be used to retrieve all current Key values;

AimRT defines some special Keys in the {{ '[rpc_context_base.h]({}/src/interface/aimrt_module_c_interface/rpc/rpc_context_base.h)'.format(code_site_root_path_url) }} file. When using these special Keys, certain rules should be followed. These special Keys include:
- **AIMRT_RPC_CONTEXT_KEY_TO_ADDR**: Used to set the peer address for RPC. The peer address should follow the standard URL format: `{{protocol}}://{{path}}`, where the `{{protocol}}` field is used by the AimRT framework to select the RPC backend, and the `{{path}}` field can be used for custom behaviors by different backends;
- **AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE**: Used to set the serialization type of the message, which must be a type supported by the registered type support;
- **AIMRT_RPC_CONTEXT_KEY_FUNCTION_NAME**: Used to pass the RPC Function name;
- **AIMRT_RPC_CONTEXT_KEY_BACKEND**: Used to pass the actual backend name to the Server side;

On the Client side, `Context` is mainly used to pass special information to the AimRT framework and RPC backend when making RPC calls. The following points should be noted when using it:
- Developers can directly construct an instance of the `Context` type and are responsible for its lifecycle;
- Only Client-type ctx can be passed to Client-side RPC call methods;
- Each `Context` can only be used for one Client-side call. After being passed to the Client-side RPC call method, its state will be set to `Used`. If used for another RPC call without `Reset`, the request will fail;
- The Client-side RPC call methods actually accept parameters of type `ContextRef`, but the `Context` type can be implicitly converted to `ContextRef`;
- Developers can set Timeout in the ctx, but the handling of Timeout depends on the actual RPC backend. Refer to the specific RPC backend's documentation for details;
- Developers can set information in the ctx to pass to specific RPC backends. Different backends may handle the information in the ctx differently—some may read specific Key-Value pairs to specialize transmission behavior, while others may pass all Key-Value information downstream. Refer to the specific RPC backend's documentation for details.

On the Server side, developers can receive parameters of type `ContextRef` in callback handler functions. The following points should be noted when using it:
- The lifecycle of the ctx passed to the callback handler function is managed by the AimRT framework and is consistent with the lifecycle of Req and Rsp;
- The ctx passed to the callback handler function is of Server type and is in the `Used` state;
- The ctx passed to the callback handler function may contain Timeout information and some Key-Value information. The specific information passed depends on the actual RPC backend. Refer to the specific RPC backend's documentation.

Additionally, in a complex business system, some servers may initiate requests to downstream services after receiving a request, forming a logical long chain. To bridge this logical chain at the framework level for monitoring and scheduling purposes, specific information from the Server-type ctx needs to be synchronized to the Client-type ctx. There are two methods:

1. Use the `MergeServerContextToClientContext` method provided by the `RpcHandleRef` type, for example:
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

2. Use the Proxy's `NewContextSharedPtr` method, passing the Server-type ctx as a parameter to this method, for example:
```cpp
ExampleServiceSyncProxy proxy;

// RPC server handle function
aimrt::rpc::Status ExampleServiceSyncServiceImpl::GetBarData(```cpp
aimrt::rpc::ContextRef server_ctx, const GetBarDataReq& bar_req, GetBarDataRsp& bar_rsp) {
  GetFooDataReq foo_req;
  GetFooDataRsp foo_rsp;

  auto client_ctx = proxy.NewContextSharedPtr(server_ctx);

  auto foo_status = proxy.GetFooData(client_ctx, req, rsp);

  // ...

  return aimrt::rpc::Status();
}
```