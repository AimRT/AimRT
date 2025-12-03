# Rpc

## Related Links

Code files:
- {{ '[aimrt_module_cpp_interface/rpc/rpc_handle.h]({}/src/interface/aimrt_module_cpp_interface/rpc/rpc_handle.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/rpc/rpc_context.h]({}/src/interface/aimrt_module_cpp_interface/rpc/rpc_context.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_c_interface/rpc/rpc_status_base.h]({}/src/interface/aimrt_module_c_interface/rpc/rpc_status_base.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/rpc/rpc_status.h]({}/src/interface/aimrt_module_cpp_interface/rpc/rpc_status.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/rpc/rpc_co_filter.h]({}/src/interface/aimrt_module_cpp_interface/rpc/rpc_co_filter.h)'.format(code_site_root_path_url) }}

Reference examples:
- {{ '[pb_rpc]({}/src/examples/cpp/pb_rpc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc]({}/src/examples/cpp/ros2_rpc)'.format(code_site_root_path_url) }}


## Protocol

The protocol is used to determine the message format between the client and server in RPC. Generally, protocols are described using an IDL (Interface Description Language) that is independent of any specific programming language, and then converted into code for each language by some tool. For RPC, two steps are required:
- Refer to the [Channel](./channel.md) chapter; developers first need to use some official tools to generate code in the specified programming language for the **message types** in the protocol file;
- Developers then need to use the tools provided by AimRT to generate code in the specified programming language for the **service definitions** in the protocol file;

### Protobuf

[Protobuf](https://protobuf.dev/) is a lightweight and efficient data exchange format developed by Google for serializing structured data and is a widely used IDL. It not only describes message structures but also provides the `service` statement to define RPC services.


When using it, developers first need to define a `.proto` file in which the message structures and RPC services are defined. For example, `rpc.proto`:


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


Then use the protoc tool provided officially by Protobuf to convert and generate the C++ code for the message structure part, for example:

```shell
protoc --cpp_out=. rpc.proto
```


This will generate `rpc.pb.h` and `rpc.pb.cc` files, containing the C++ classes and methods generated according to the defined message types.


After that, you also need to use the protoc plugin provided by AimRT to generate the C++ code for the service definition part, for example:

```shell
protoc --aimrt_rpc_out=. --plugin=protoc-gen-aimrt_rpc=./protoc_plugin_py_gen_aimrt_cpp_rpc.py rpc.proto
```


This will generate `rpc.aimrt_rpc.pb.h` and `rpc.aimrt_rpc.pb.cc` files, containing the C++ classes and methods generated according to the defined services.

Please note that the above native code generation method is only to show developers the underlying principles; in actual use, you still need to manually handle dependencies and CMake packaging, which is rather cumbersome. AimRT has encapsulated this process to some extent, and developers can directly use the CMake methods contained in the following two files:


1. {{ '[ProtobufGenCode.cmake]({}/cmake/ProtobufGenCode.cmake)'.format(code_site_root_path_url) }}: Used to generate the C++ code for the message structure part, which contains two CMake methods:
- `add_protobuf_gencode_target_for_proto_path`: Generates C++ code for `.proto` files in a certain path, with parameters as follows:
  - **TARGET_NAME**: The name of the generated CMake Target;
  - **PROTO_PATH**: The directory where the protocol files are stored;
  - **GENCODE_PATH**: The path where the generated stub code is stored;
  - **DEP_PROTO_TARGETS**: The dependent Proto CMake Target;
  - **OPTIONS**: Other parameters passed to protoc;
- `add_protobuf_gencode_target_for_one_proto_file`: Generates C++ code for a single `.proto` file, with parameters as follows:
  - **TARGET_NAME**: The name of the generated CMake Target;
  - **PROTO_FILE**: The path to a single protocol file;
  - **GENCODE_PATH**: The path where the generated stub code is stored;
  - **DEP_PROTO_TARGETS**: The dependent Proto CMake Target;
  - **OPTIONS**: Other parameters passed to protoc;


2. {{ '[ProtobufAimRTRpcGenCode.cmake]({}/src/tools/protoc_plugin_cpp_gen_aimrt_cpp_rpc/ProtobufAimRTRpcGenCode.cmake)'.format(code_site_root_path_url) }}: Used to generate C++ service code, which contains one CMake method:
- `add_protobuf_aimrt_rpc_gencode_target_for_proto_files`: Generates C++ service code for some `.proto` files, with parameters as follows:
  - **TARGET_NAME**: The name of the generated CMake Target;
  - **PROTO_FILES**: The paths to the protocol files;
  - **GENCODE_PATH**: The path where the generated stub code is stored;
  - **DEP_PROTO_TARGETS**: The dependent Proto CMake Target;
  - **OPTIONS**: Other parameters passed to protoc;



The above methods need to be used in combination: first generate the C++ code for the message structure part, then generate the C++ service code. Example as follows:
```cmake
# Generate C++ code for all '.proto' files in the current folder
add_protobuf_gencode_target_for_proto_path(
  TARGET_NAME example_pb_gencode
  PROTO_PATH ${CMAKE_CURRENT_SOURCE_DIR}
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR})
```# Generate RPC service C++ code for the 'rpc.proto' file. Needs to depend on 'example_pb_gencode'
add_protobuf_aimrt_rpc_gencode_target_for_proto_files(
  TARGET_NAME example_rpc_aimrt_rpc_gencode
  PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/rpc.proto
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS example_pb_gencode)

```

After that, just link the `example_rpc_aimrt_rpc_gencode` CMake Target to use the protocol. For example:

```
cmake
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

```
cmake
rosidl_generate_interfaces(
  example_srv_gencode
  "srv/example.srv"
)

```

After that, you can reference the relevant CMake Target to use the generated C++ code for Req and Rsp message structures. For details, please refer to the official ROS2 documentation and the Example provided by AimRT.


After generating the C++ code for Req and Rsp message structures, developers also need to use the Python script tool provided by AimRT to generate the C++ stub code for the service definition part, for example:


```
shell
python3 ARGS ./ros2_py_gen_aimrt_cpp_rpc.py --pkg_name=example_pkg --srv_file=./example.srv --output_path=./

```

This will generate the files `example.aimrt_rpc.srv.h` and `example.aimrt_rpc.srv.cc`, which contain the C++ classes and methods generated according to the defined service.


Note that the above process for generating C++ service code from ROS2 is mainly to demonstrate the underlying principles to developers. In actual use, you still need to manually handle dependencies and CMake integration, which can be cumbersome. AimRT provides some encapsulation for this process, allowing developers to directly use the CMake methods provided in the {{ '[Ros2AimRTRpcGenCode.cmake]({}/src/tools/ros2_py_gen_aimrt_cpp_rpc/Ros2AimRTRpcGenCode.cmake)'.format(code_site_root_path_url) }} file:

- `add_ros2_aimrt_rpc_gencode_target_for_one_file`: Generates RPC service C++ code for a single `.srv` file. The parameters are as follows:
  - **TARGET_NAME**: The name of the generated CMake Target;
  - **PACKAGE_NAME**: The name of the ROS2 protocol package (PKG);
  - **PROTO_FILE**: The path to the protocol file;
  - **GENCODE_PATH**: The path where the generated stub code will be stored;
  - **DEP_PROTO_TARGETS**: Dependent protocol CMake Targets;
  - **OPTIONS**: Other parameters to pass to the tool;


In actual use, you need to first generate the C++ code for the message structures, and then generate the C++ service code. Here is an example:

```
cmake
# Generate C++ code for Req and Rsp message in `.srv` file
rosidl_generate_interfaces(
  example_srv_gencode
  "srv/example.srv"
)

# Generate RPC service C++ code for the example '.srv' file. It is necessary to depend on the CMake Target related to ROS2 messages, which is defined in '${ROS2_EXAMPLE_CMAKE_TARGETS}'
add_ros2_aimrt_rpc_gencode_target_for_one_file(
  TARGET_NAME example_ros2_rpc_aimrt_rpc_gencode
  PACKAGE_NAME example_pkg
  PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/srv/example.srv
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS
    rclcpp::rclcpp
    ${ROS2_EXAMPLE_CMAKE_TARGETS})

```

Afterwards, you only need to link the `example_ros2_rpc_aimrt_rpc_gencode` CMake Target to use this protocol. For example:

```
cmake
target_link_libraries(my_lib PUBLIC example_ros2_rpc_aimrt_rpc_gencode)
```

## RpcHandle

In AimRT, a module can obtain the `aimrt::rpc::RpcHandleRef` handle by calling the `GetRpcHandle()` interface of the `CoreRef` handle. Generally, developers do not directly use the interfaces provided by `aimrt::rpc::RpcHandleRef`, but instead generate some stub code based on the RPC IDL file, wrap the `RpcHandleRef` handle, and then use these wrapped interfaces in business code.

The specific form of these wrapped interfaces will be introduced in subsequent sections of this document. When using RPC functionality, developers need to follow these steps to use these interfaces:
- Client side:
  - During the `Initialize` phase, call the interface to **register the RPC Client method**;
  - During the `Start` phase, call the **RPC Invoke** interface to perform RPC calls;
- Server side:
  - During the `Initialize` phase, call the interface to **register the RPC Server service**;

AimRT officially supports two protocol IDLs: **Protobuf** and **Ros2 Srv**, and provides tools to generate stub code for these two protocol IDLs. The generated RPC interfaces have consistent API styles except for the protocol type.

Developers can also use the `MergeServerContextToClientContext` method to pass server-side context information to the client-side context, which can be used to link the entire data chain. For details, please refer to the Context chapter.## Status

During an RPC call or while handling an RPC, users can obtain error information via an `aimrt::rpc::Status` variable. Its interface is as follows:

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


The `Status` type is extremely lightweight, containing only an error-code field. Users can set this code via the constructor or a Set method, and retrieve it via a Get method. The enumerated error-code values are defined in {{ '[rpc_status_base.h]({}/src/interface/aimrt_module_c_interface/rpc/rpc_status_base.h)'.format(code_site_root_path_url) }} and are listed below:

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
| Server | `AIMRT_RPC_STATUS_SVR_HANDLE_FAILED` | 1008 | Handling failed |
| Server | `AIMRT_RPC_STATUS_SVR_NOT_READY` | 1009 | Server not started (context interface closed) |
| Client | `AIMRT_RPC_STATUS_CLI_UNKNOWN` | 2000 | Client-side unknown error |
| Client | `AIMRT_RPC_STATUS_CLI_FUNC_NOT_REGISTERED` | 2001 | Function not registered |
| Client | `AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR` | 2002 | Client-side internal error |
| Client | `AIMRT_RPC_STATUS_CLI_INVALID_CONTEXT` | 2003 | Invalid context |
| Client | `AIMRT_RPC_STATUS_CLI_INVALID_ADDR` | 2004 | Invalid address |
| Client | `AIMRT_RPC_STATUS_CLI_INVALID_SERIALIZATION_TYPE` | 2005 | Invalid serialization type |
| Client | `AIMRT_RPC_STATUS_CLI_SERIALIZATION_FAILED` | 2006 | Serialization failed |
| Client | `AIMRT_RPC_STATUS_CLI_INVALID_DESERIALIZATION_TYPE` | 2007 | Invalid deserialization type |
| Client | `AIMRT_RPC_STATUS_CLI_DESERIALIZATION_FAILED` | 2008 | Deserialization failed |
| Client | `AIMRT_RPC_STATUS_CLI_NO_BACKEND_TO_HANDLE` | 2009 | No backend to handle |
| Client | `AIMRT_RPC_STATUS_CLI_SEND_REQ_FAILED` | 2010 | Request sending failed |
| Client | `AIMRT_RPC_STATUS_CLI_NOT_READY` | 2011 | Client not started (context interface closed) |

Please note that the error information in `Status` generally reflects framework-level errors—such as service not found, network issues, or serialization failures—and is intended for developers to diagnose framework-level problems. If developers need to return business-level errors, it is recommended to add corresponding fields in the business message.

Additionally, when using the **ROS2 RPC backend in combination with ROS2 Srv**, because ROS2 itself does not support returning any fields other than request_id and response, the framework will not return the error code provided by the server side; instead, it will simply return `AIMRT_RPC_STATUS_OK`.
For example, if a service is not implemented on the server side and should return `AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED`, due to the limitations of the above combination, the framework will only return `AIMRT_RPC_STATUS_OK` to the client.

## Client

In the stub code generated by the AimRT RPC tool, such as `rpc.aimrt_rpc.pb.h` or `example.aimrt_rpc.srv.h`, four types of Client Proxy interfaces are provided. Developers use these Proxy interfaces to initiate RPC calls:
- **Synchronous interface**: typically named `XXXSyncProxy`;
- **Asynchronous callback interface**: typically named `XXXAsyncProxy`;
- **Asynchronous Future interface**: typically named `XXXFutureProxy`;
- **Stackless coroutine interface**: typically named `XXXCoProxy`;

These Proxy types can be mixed and matched; developers can choose the appropriate type according to their needs. Aside from differing API interfaces when invoking RPCs, their underlying runtime behavior is identical.### Public Interface

All Proxies have a common base class and share some public interfaces, as shown below:

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


Specific descriptions:
- Proxies are generally lightweight and can be created on demand;
- RPC Type is an inherent attribute indicating the system to which the RPC Service name belongs, such as `pb`, `ros2`, etc. It can be obtained through the `RpcType` method;
- RPC ServiceName denotes the name of this RPC service. If no special configuration is provided, a default value bound to the **protocol name** will be used. If you need to provide different services using the same protocol, you can also set it via the `SetServiceName` method;
- All types of Proxies must be constructed from an `aimrt::rpc::RpcHandleRef` handle;
- All types of Proxies provide a static method `RegisterClientFunc` for registering the current RPC Client:
  - This method actually calls a globally generated `RegisterXXXClientFunc` for the current RPC;
  - It requires passing an `aimrt::rpc::RpcHandleRef` handle as a parameter;
  - It can optionally accept an RPC ServiceName field to be used as the RPC service name during registration;
- If there are multiple Proxies of the same type, they are distinguished by `ServiceName`. Developers must ensure that the `ServiceName` used during registration matches the one used at runtime;
- A default Context can be set for a Proxy:
  - If no Context is provided during an RPC call or an empty Context is provided, the Proxy's default Context will be used;
  - Users can set and get the default Context via `SetDefaultContextSharedPtr` and `GetDefaultContextSharedPtr`;
  - Users can obtain a new Context copied from the default Context using the `NewContextSharedPtr` method;



### Synchronous Interface

The synchronous interface is the simplest to use but has the lowest runtime efficiency. It blocks the current thread and waits for the RPC interface to return. This approach can be used in scenarios where performance is not critical to improve development efficiency, but it is not recommended for high-performance requirements.


Using the synchronous interface to initiate an RPC call is very straightforward and generally involves the following steps:
- **Step 0**: Include the stub header file, such as `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contains the synchronous interface handle `XXXSyncProxy`;
- **Step 1**: During the `Initialize` phase, call the `RegisterClientFunc` method to register the RPC Client;
- **Step 2**: In a business function during the `Start` phase, initiate the RPC call:
  - **Step 2-1**: Create an `XXXSyncProxy`, with the constructor parameter being an `aimrt::rpc::RpcHandleRef` handle;
  - **Step 2-2**: Create Req and Rsp, and populate the Req content;
  - **Step 2-3**: [Optional] Create ctx and set timeout and other information;
  - **Step 2-4**: Use the proxy to pass ctx, Req, and Rsp, initiate the RPC call, and synchronously wait for the RPC call to complete. Ensure that ctx, Req, and Rsp remain valid and unchanged throughout the call lifecycle, and finally obtain the returned status;
  - **Step 2-5**: Parse the status and Rsp;


Below is a simple example based on protobuf; the syntax for ROS2 Srv is basically similar:

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
- {{ '[ros2_rpc_sync_client]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_sync_client_module/normal_rpc_sync_client_module.cc)'.format(code_site_root_path_url) }}### Asynchronous Callback Interface

The asynchronous callback interface uses callbacks to return asynchronous results. It offers the best performance but is the least developer-friendly and can easily lead to callback hell.

Initiating an RPC call with the asynchronous callback interface generally involves the following steps:
- **Step 0**: Include the stub header file, such as `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contains the asynchronous interface handle `XXXAsyncProxy`;
- **Step 1**: During the `Initialize` phase, call the `RegisterClientFunc` method to register the RPC Client;
- **Step 2**: In a business function during the `Start` phase, initiate the RPC call:
  - **Step 2-1**: Create an `XXXAsyncProxy`, with the constructor parameter being `aimrt::rpc::RpcHandleRef`;
  - **Step 2-2**: Create Req and Rsp, and populate the Req content;
  - **Step 2-3**: [Optional] Create ctx and set timeout and other information;
  - **Step 2-4**: Using the proxy, pass ctx, Req, Rsp, and the result callback to initiate the RPC call, ensuring that ctx, Req, and Rsp remain valid and unchanged throughout the entire call lifecycle;
  - **Step 2-5**: In the callback function, retrieve the returned status and parse the status and Rsp;

The first few steps are basically the same as the synchronous interface, with the difference being that **Step 2-4** requires using an asynchronous callback to obtain the result. Below is a simple protobuf-based example; the syntax based on ROS2 Srv is also very similar:

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


The asynchronous Future-style interface returns asynchronous results based on `std::future`. After initiating an RPC call, developers can do other things first and then block to obtain the result by calling `std::future::get` when the RPC result is needed. It balances performance and development friendliness to some extent, serving as a middle ground between synchronous and asynchronous callback styles.


Using the asynchronous Future-style interface to initiate an RPC call generally involves the following steps:
- **Step 0**: Include the stub header file, such as `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contains the asynchronous interface handle `XXXFutureProxy`;
- **Step 1**: During the `Initialize` phase, call the `RegisterClientFunc` method to register the RPC Client;
- **Step 2**: In some business function during the `Start` phase, initiate the RPC call:
  - **Step 2-1**: Create a `XXXFutureProxy`, whose constructor parameter is `aimrt::rpc::RpcHandleRef`;
  - **Step 2-2**: Create Req and Rsp, and populate the Req content;
  - **Step 2-3**: [Optional] Create ctx and set timeout and other information;
  - **Step 2-4**: Use the proxy to initiate the RPC call by passing ctx, Req, Rsp, and the result callback, ensuring that ctx, Req, and Rsp remain valid and unchanged throughout the entire call cycle, and obtain a `std::future<Status>` handle;
  - **Step 2-5**: At a later time, blockingly call the `get()` method of the `std::future<Status>` handle to obtain the status value and parse the status and Rsp;



Below is a simple example based on protobuf; the syntax based on ROS2 Srv is basically similar:

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



For more examples, please refer to:
- {{ '[pb_rpc_future_client]({}/src/examples/cpp/pb_rpc/module/normal_rpc_future_client_module/normal_rpc_future_client_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_future_client]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_future_client_module/normal_rpc_future_client_module.cc)'.format(code_site_root_path_url) }}### Stackless Coroutine Interface


AimRT provides a stackless coroutine-style interface for the RPC Client side, implemented using C++20 coroutines and the current implementation library [libunifex](https://github.com/facebookexperimental/libunifex) of the [C++ executors proposal](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p0443r14.html). The stackless coroutine interface is essentially a wrapper around the asynchronous callback interface, offering performance nearly identical to the asynchronous callback interface while significantly improving developer-friendliness.

Initiating an RPC call using the coroutine-style interface generally involves the following steps:
- **Step 0**: Include the stub code header file, such as `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contains the coroutine interface handle `XXXCoProxy`;
- **Step 1**: During the `Initialize` phase, call the `RegisterClientFunc` method to register the RPC Client;
- **Step 2**: Within a business coroutine during the `Start` phase, initiate the RPC call:
  - **Step 2-1**: Create an `XXXCoProxy`, with the constructor parameter being `aimrt::rpc::RpcHandleRef`;
  - **Step 2-2**: Create Req and Rsp, and populate the Req content;
  - **Step 2-3**: [Optional] Create ctx and set timeout and other information;
  - **Step 2-4**: Using the proxy, pass ctx, Req, Rsp, and the result callback to initiate the RPC call, wait for the RPC call to complete within the coroutine, ensuring that ctx, Req, and Rsp remain valid and unchanged throughout the entire call cycle, and obtain the returned status;
  - **Step 2-5**: Parse the status and Rsp;

The overall interface style is almost identical to the synchronous interface, but it must be called within a coroutine. Below is a simple example based on protobuf; the syntax based on ROS2 Srv is basically similar:

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



For more examples, please refer to:
- {{ '[pb_rpc_co_client]({}/src/examples/cpp/pb_rpc/module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_co_client]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)'.format(code_site_root_path_url) }}


## Server

In the code generated by the AimRT RPC stub code tool, such as `rpc.aimrt_rpc.pb.h` or `example.aimrt_rpc.srv.h`, three types of Service base classes are provided. Developers inherit these Service base classes and implement the virtual interfaces within them to provide actual RPC services:
- **Synchronous Interface**: Generally named `XXXSyncService`;
- **Asynchronous Callback Interface**: Generally named `XXXAsyncService`;
- **Stackless Coroutine Interface**: Generally named `XXXCoService`;

Within a single service, these three types cannot be mixed; only one can be chosen. Developers can select based on their own needs.


### Common Interface

All Services have a common base class that shares some common interfaces, as shown below:

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


Specific explanations are as follows:
- Developers need to inherit these Service base classes to implement business logic. Developers must manage the lifecycle of the business Service instances themselves;
- RPC Type is an inherent attribute, indicating the system to which the RPC Service name belongs, such as `pb`, `ros2`, etc. It can be obtained via the `RpcType` method;
- RPC ServiceName indicates the name of the RPC service. If no special configuration is made, a default value bound to the **protocol name** will be used. If you need to provide different services using the same protocol, you can also set it via the `SetServiceName` method;



Best practice: If the task within the callback is very lightweight, such as just setting a variable, it can be handled directly within the callback; however, if the task within the callback is relatively heavy, it is best to schedule it to another executor specifically designed for handling tasks.### Synchronous Interface

The synchronous interface is the simplest to use, but in many cases the business RPC handler needs to make further downstream calls, which may involve asynchronous invocations. In such scenarios, the handler can only block and wait for the downstream call to complete, potentially reducing runtime efficiency. It is generally suitable for handling simple requests that do not require initiating other asynchronous calls.

To implement an RPC service using the synchronous interface, the process is generally divided into the following steps:
- **Step 0**: Include the stub header file, e.g., `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contains the synchronous service base class `XXXSyncService`;
- **Step 1**: The developer implements an Impl class that inherits from `XXXSyncService` and overrides its virtual functions;
  - **Step 1-1**: Parse the Req and fill the Rsp;
  - **Step 1-2**: Return a `Status`;
- **Step 2**: During the `Initialize` phase, call the `RegisterService` method of `RpcHandleRef` to register the RPC Service;

Below is a simple protobuf-based example; the ROS2 Srv syntax is almost identical:

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


For more examples, please refer to:
- {{ '[pb_rpc_sync_service]({}/src/examples/cpp/pb_rpc/module/normal_rpc_sync_server_module/service.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_sync_service]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_sync_server_module/service.cc)'.format(code_site_root_path_url) }}

### Asynchronous Callback Interface

The asynchronous callback interface passes a callback to the developer, who invokes this callback upon completion of the RPC to deliver the final result. This approach allows initiating other asynchronous calls within the RPC, and because it does not block, it usually delivers the best performance. However, it often leads to code that is harder to read and maintain.

To implement an RPC service using the asynchronous callback interface, the process is generally divided into the following steps:
- **Step 0**: Include the stub header file, e.g., `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contains the asynchronous service base class `XXXAsyncService`;
- **Step 1**: The developer implements an Impl class that inherits from `XXXAsyncService` and overrides its virtual functions;
  - **Step 1-1**: Parse the Req and fill the Rsp;
  - **Step 1-2**: Invoke the callback to pass back the `Status`;
- **Step 2**: During the `Initialize` phase, call the `RegisterService` method of `RpcHandleRef` to register the RPC Service;

Below is a simple protobuf-based example; the ROS2 Srv syntax is almost identical:

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


For more examples, please refer to:
- {{ '[pb_rpc_async_service]({}/src/examples/cpp/pb_rpc/module/normal_rpc_async_server_module/service.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_async_service]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_async_server_module/service.cc)'.format(code_site_root_path_url) }}### Stackless Coroutine Interface

Just like on the RPC Client side, on the RPC Service side AimRT also provides a stackless coroutine-style interface implemented with C++20 coroutines and the current implementation library [libunifex](https://github.com/facebookexperimental/libunifex) of the [C++ executors proposal](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p0443r14.html). The stackless coroutine interface is essentially a wrapper around the asynchronous callback interface; its performance is almost identical to the asynchronous callback interface, while greatly improving developer friendliness.

Implementing an RPC service with the coroutine-style interface generally involves the following steps:
- **Step 0**: Include the stub header file, e.g., `xxx.aimrt_rpc.pb.h` or `xxx.aimrt_rpc.srv.h`, which contains the coroutine interface’s Service base class `XXXCoService`;
- **Step 1**: The developer implements an Impl class that inherits from `XXXCoService` and implements its virtual interfaces;
  - **Step 1-1**: Parse the Req and fill the Rsp;
  - **Step 1-2**: Use `co_return` to return a `Status`;
- **Step 2**: During the `Initialize` phase, call the `RegisterService` method of `RpcHandleRef` to register the RPC Service;

The overall interface style is almost identical to the synchronous interface. Below is a simple protobuf-based example; the syntax based on ROS2 Srv is very similar:

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

When developers invoke an RPC, they can pass an `aimrt::rpc::Context`; when processing an RPC, they will receive an `aimrt::rpc::ContextRef`. `ContextRef` is a reference type to `Context`, and the two expose almost the same interface. Their main purpose is to carry a timeout configuration and some key-value data, so that specific information can be propagated downstream or to the RPC backend.

The interface is shown below:


```cpp
namespace aimrt::rpc {

class Context {
 public:
  explicit Context(ContextRef ref);

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


When using the RPC ctx of type `Context` or `ContextRef`, note the following:
- RPC ctx is divided into Client-side and Server-side types, determined at construction and immutable; they are used in Client and Server scenarios respectively;
- Use `SetTimeout` and `Timeout` to set and retrieve the timeout configuration in ctx;
- Use `SetMetaValue` and `GetMetaValue` to set and retrieve key-value pairs in ctx, and use `GetMetaKeys` to obtain all current keys;

AimRT defines some special keys in {{ '[rpc_context_base.h]({}/src/interface/aimrt_module_c_interface/rpc/rpc_context_base.h)'.format(code_site_root_path_url) }}. When business code uses these special keys, certain rules must be followed. These special keys include:
- **AIMRT_RPC_CONTEXT_KEY_TO_ADDR**: used to set the peer address of the RPC; the peer address must follow the standard URL format: `{{protocol}}://{{path}}`, where the `{{protocol}}` field is used by the AimRT framework to select the RPC backend, and the `{{path}}` field can be used for backend-specific behaviors;
- **AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE**: used to set the serialization type of the message; it must be one of the types supported in the registered type support;
- **AIMRT_RPC_CONTEXT_KEY_FUNCTION_NAME**: used to pass the RPC Function name;
- **AIMRT_RPC_CONTEXT_KEY_BACKEND**: used to pass the actual backend name to the Server side;


On the Client side, `Context` is mainly used to pass special information to the AimRT framework and the RPC backend when invoking an RPC. The following points should be noted:
- Developers can directly construct an instance of type `Context` and manage its lifecycle themselves;
- Only a Client-type ctx can be passed to the Client-side RPC invocation method;
- Each `Context` can only be used for one Client-side call; after being passed to the Client-side RPC invocation method, its state is set to `Used`. If it is used for the next RPC call without being `Reset`, the request will fail;
- The Client-side RPC invocation method actually accepts a parameter of type `ContextRef`, but the `Context` type can be implicitly converted to `ContextRef`;
- Developers can set a timeout in ctx, but how the timeout is handled depends on the actual RPC backend; please refer to the documentation of the specific RPC backend.
- Developers can set some information in ctx to be passed to the specific RPC backend; different backends handle the information in ctx differently—some read specific key-value pairs to specialize transport behavior, while others transparently pass all key-value information downstream; please refer to the documentation of the specific RPC backend.



On the Server side, developers can receive a parameter of type `ContextRef` in the callback handler. The following points should be noted:
- The lifecycle of the ctx passed to the callback handler is managed by the AimRT framework and is consistent with the lifecycle of Req and Rsp;
- The ctx passed to the callback handler is of Server type and is in the `Used` state;
- The ctx passed to the callback handler may contain timeout information and some key-value information; which information is actually passed is determined by the actual RPC backend—please refer to the documentation of the specific RPC backend.



In addition, in a complex business system, some services may, after receiving a request, initiate further requests to even more downstream services, thus forming a logically long chain. To connect this logical chain at the framework level for monitoring and scheduling purposes, specific information from the Server-type ctx needs to be synchronized to a Client-type ctx. There are two ways to do this:


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


2. Use the `NewContextSharedPtr` method of the Proxy, passing the Server-type ctx as a parameter to this method, for example:
```cpp
ExampleServiceSyncProxy proxy;

// RPC server handle function
aimrt::rpc::Status ExampleServiceSyncServiceImpl::GetBarData(
```    aimrt::rpc::ContextRef server_ctx, const GetBarDataReq& bar_req, GetBarDataRsp& bar_rsp) {
  GetFooDataReq foo_req;
  GetFooDataRsp foo_rsp;

  auto client_ctx = proxy.NewContextSharedPtr(server_ctx);

  auto foo_status = proxy.GetFooData(client_ctx, req, rsp);

  // ...

  return aimrt::rpc::Status();
}
```
