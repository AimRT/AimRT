# Rpc

## 相关链接

代码文件：
- {{ '[aimrt_module_cpp_interface/rpc/rpc_handle.h]({}/src/interface/aimrt_module_cpp_interface/rpc/rpc_handle.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/rpc/rpc_context.h]({}/src/interface/aimrt_module_cpp_interface/rpc/rpc_context.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_c_interface/rpc/rpc_status_base.h]({}/src/interface/aimrt_module_c_interface/rpc/rpc_status_base.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/rpc/rpc_status.h]({}/src/interface/aimrt_module_cpp_interface/rpc/rpc_status.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/rpc/rpc_co_filter.h]({}/src/interface/aimrt_module_cpp_interface/rpc/rpc_co_filter.h)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[pb_rpc]({}/src/examples/cpp/pb_rpc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc]({}/src/examples/cpp/ros2_rpc)'.format(code_site_root_path_url) }}


## 协议

协议用于确定 RPC 中客户端和服务端的消息格式。一般来说，协议都是使用一种与具体的编程语言无关的 IDL ( Interface description language )描述，然后由某种工具转换为各个语言的代码。对于 RPC 来说，这里需要两个步骤：
- 参考[Channel](./channel.md)章节中的介绍，开发者需要先利用一些官方的工具为协议文件中的**消息类型**生成指定编程语言中的代码；
- 开发者需要使用 AimRT 提供的工具，为协议文件中**服务定义**生成指定编程语言中的代码；

### Protobuf

[Protobuf](https://protobuf.dev/)是一种由 Google 开发的、用于序列化结构化数据的轻量级、高效的数据交换格式，是一种广泛使用的 IDL。它不仅能够描述消息结构，还提供了`service`语句来定义 RPC 服务。


在使用时，开发者需要先定义一个`.proto`文件，在其中定义消息结构和 RPC 服务。例如`rpc.proto`：

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

然后使用 Protobuf 官方提供的 protoc 工具进行转换，生成消息结构部分的 C++ 代码，例如：
```shell
protoc --cpp_out=. rpc.proto
```

这将生成`rpc.pb.h`和`rpc.pb.cc`文件，包含了根据定义的消息类型生成的 C++ 类和方法。


在这之后，还需要使用 AimRT 提供的 protoc 插件，生成服务定义部分的 C++ 代码，例如：
```shell
protoc --aimrt_rpc_out=. --plugin=protoc-gen-aimrt_rpc=./protoc_plugin_py_gen_aimrt_cpp_rpc.py rpc.proto
```

这将生成`rpc.aimrt_rpc.pb.h`和`rpc.aimrt_rpc.pb.cc`文件，包含了根据定义的服务生成的 C++ 类和方法。

请注意，以上这套原生的代码生成方式只是为了给开发者展示底层的原理，实际使用时还需要手动处理依赖和 CMake 封装等方面的问题，比较繁琐。AimRT 对这个过程进行了一定的封装，开发者可以直接使用以下两个文件中包含的 CMake 方法：


1. {{ '[ProtobufGenCode.cmake]({}/cmake/ProtobufGenCode.cmake)'.format(code_site_root_path_url) }}：用于生成消息结构体部分的 C++ 代码，其中包含两个 CMake 方法：
- `add_protobuf_gencode_target_for_proto_path`：为某个路径下的`.proto`文件生成 C++ 代码，参数如下：
  - **TARGET_NAME**：生成的 CMake Target 名称；
  - **PROTO_PATH**：协议存放目录；
  - **GENCODE_PATH**：生成的桩代码存放路径；
  - **DEP_PROTO_TARGETS**：依赖的 Proto CMake Target；
  - **OPTIONS**：传递给 protoc 的其他参数；
- `add_protobuf_gencode_target_for_one_proto_file`：为单个`.proto`文件生成 C++ 代码，参数如下：
  - **TARGET_NAME**：生成的 CMake Target 名称；
  - **PROTO_FILE**：单个协议文件的路径；
  - **GENCODE_PATH**：生成的桩代码存放路径；
  - **DEP_PROTO_TARGETS**：依赖的 Proto CMake Target；
  - **OPTIONS**：传递给 protoc 的其他参数；


2. {{ '[ProtobufAimRTRpcGenCode.cmake]({}/src/tools/protoc_plugin_cpp_gen_aimrt_cpp_rpc/ProtobufAimRTRpcGenCode.cmake)'.format(code_site_root_path_url) }}：用于生成 C++ 服务代码，其中包含一个 CMake 方法：
- `add_protobuf_aimrt_rpc_gencode_target_for_proto_files`：为一些`.proto`文件生成 C++ 服务代码，参数如下：
  - **TARGET_NAME**：生成的 CMake Target 名称；
  - **PROTO_FILES**：协议文件的路径；
  - **GENCODE_PATH**：生成的桩代码存放路径；
  - **DEP_PROTO_TARGETS**：依赖的 Proto CMake Target；
  - **OPTIONS**：传递给 protoc 的其他参数；



以上方法需要结合使用，先生成消息结构体部分的 C++ 代码，再生成 C++ 服务代码。示例如下：
```cmake
# Generate C++ code for all '.proto' files in the current folder
add_protobuf_gencode_target_for_proto_path(
  TARGET_NAME example_pb_gencode
  PROTO_PATH ${CMAKE_CURRENT_SOURCE_DIR}
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR})

# Generate RPC service C++ code for 'rpc.proto' file. Need to rely on 'example_pb_gencode'
add_protobuf_aimrt_rpc_gencode_target_for_proto_files(
  TARGET_NAME example_rpc_aimrt_rpc_gencode
  PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/rpc.proto
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR}
  DEP_PROTO_TARGETS example_pb_gencode)
```

之后只要链接`example_rpc_aimrt_rpc_gencode`这个 CMake Target 即可使用该协议。例如：
```cmake
target_link_libraries(my_lib PUBLIC example_rpc_aimrt_rpc_gencode)
```

### ROS2 Srv

ROS2 Srv 是一种用于在 ROS2 中进行 RPC 定义的格式。在使用时，开发者需要先定义一个 ROS2 Package，在其中定义一个`.srv`文件，比如`example.srv`：

```
byte[]  data
---
int64   code
```

其中，以`---`来分割 Req 和 Rsp 的定义。然后直接通过 ROS2 提供的 CMake 方法`rosidl_generate_interfaces`，为 Req 和 Rsp 消息生成 C++ 代码和 CMake Target，例如：
```cmake
rosidl_generate_interfaces(
  example_srv_gencode
  "srv/example.srv"
)
```

之后就可以引用相关的 CMake Target 来使用生成的 Req 和 Rsp 的消息结构 C++ 代码。详情请参考 ROS2 的官方文档和 AimRT 提供的 Example。

在生成了 Req 和 Rsp 消息结构的 C++ 代码后，开发者还需要使用 AimRT 提供的 Python 脚本工具，生成服务定义部分的 C++ 桩代码，例如：
```shell
python3 ARGS ./ros2_py_gen_aimrt_cpp_rpc.py --pkg_name=example_pkg --srv_file=./example.srv --output_path=./
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

之后只要链接`example_ros2_rpc_aimrt_rpc_gencode`这个 CMake Target 即可使用该协议。例如：
```cmake
target_link_libraries(my_lib PUBLIC example_ros2_rpc_aimrt_rpc_gencode)
```

## RpcHandle

AimRT 中，模块可以通过调用`CoreRef`句柄的`GetRpcHandle()`接口，获取`aimrt::rpc::RpcHandleRef`句柄。一般情况下，开发者不会直接使用`aimrt::rpc::RpcHandleRef`直接提供的接口，而是根据 RPC IDL 文件生成一些桩代码，对`RpcHandleRef`句柄做一些封装，然后在业务代码中使用这些经过封装的接口。


这些经过封装的接口的具体形式将在本文档后续章节介绍。开发者在使用 RPC 功能时需要按照以下步骤使用这些接口：
- Client 端：
  - 在`Initialize`阶段，调用**注册 RPC Client 方法**的接口；
  - 在`Start`阶段，调用 **RPC Invoke** 的接口，以实现 RPC 调用；
- Server 端：
  - 在`Initialize`阶段，**注册 RPC Server 服务**的接口；


AimRT 官方支持两种协议 IDL：**Protobuf**和**Ros2 Srv**，并提供了针对这两种协议 IDL 生成桩代码的工具。生成出来的 RPC 接口除了协议类型不同，其他的 Api 风格都一致。

开发者还可以使用`MergeServerContextToClientContext`方法，来将 server 端的 context 信息传递到 client 端的 context 中，可以用于打通整条数据链路。详情请参考 Context 章节的说明。

## Status

在 RPC 调用或者 RPC 处理时，使用者可以通过一个`aimrt::rpc::Status`类型的变量获取 RPC 过程中的错误情况，其包含的接口如下：
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


`Status`类型非常轻量，其中只包含一个错误码字段。使用者可以通过构造函数或 Set 方法设置这个 Code，可以通过 Get 方法获取这个 Code。错误码的枚举值在{{ '[rpc_status_base.h]({}/src/interface/aimrt_module_c_interface/rpc/rpc_status_base.h)'.format(code_site_root_path_url) }}文件中可以找到，列举如下：

| 错误码类型 | 错误码 | 值 | 描述 |
|------|--------|----|------|
| 通用 | `AIMRT_RPC_STATUS_OK` | 0 | 操作成功 |
| 通用 | `AIMRT_RPC_STATUS_UNKNOWN` | 1 | 未知错误 |
| 通用 | `AIMRT_RPC_STATUS_TIMEOUT` | 2 | 超时 |
| 服务端 | `AIMRT_RPC_STATUS_SVR_UNKNOWN` | 1000 | 服务器端未知错误 |
| 服务端 | `AIMRT_RPC_STATUS_SVR_BACKEND_INTERNAL_ERROR` | 1001 | 服务器端内部错误 |
| 服务端 | `AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED` | 1002 | 服务未实现 |
| 服务端 | `AIMRT_RPC_STATUS_SVR_NOT_FOUND` | 1003 | 服务未找到 |
| 服务端 | `AIMRT_RPC_STATUS_SVR_INVALID_SERIALIZATION_TYPE` | 1004 | 无效的序列化类型 |
| 服务端 | `AIMRT_RPC_STATUS_SVR_SERIALIZATION_FAILED` | 1005 | 序列化失败 |
| 服务端 | `AIMRT_RPC_STATUS_SVR_INVALID_DESERIALIZATION_TYPE` | 1006 | 无效的反序列化类型 |
| 服务端 | `AIMRT_RPC_STATUS_SVR_DESERIALIZATION_FAILED` | 1007 | 反序列化失败 |
| 服务端 | `AIMRT_RPC_STATUS_SVR_HANDLE_FAILED` | 1008 | 处理失败 |
| 客户端 | `AIMRT_RPC_STATUS_CLI_UNKNOWN` | 2000 | 客户端未知错误 |
| 客户端 | `AIMRT_RPC_STATUS_CLI_FUNC_NOT_REGISTERED` | 2001 | 函数未注册 |
| 客户端 | `AIMRT_RPC_STATUS_CLI_BACKEND_INTERNAL_ERROR` | 2002 | 客户端内部错误 |
| 客户端 | `AIMRT_RPC_STATUS_CLI_INVALID_CONTEXT` | 2003 | 无效的上下文 |
| 客户端 | `AIMRT_RPC_STATUS_CLI_INVALID_ADDR` | 2004 | 无效的地址 |
| 客户端 | `AIMRT_RPC_STATUS_CLI_INVALID_SERIALIZATION_TYPE` | 2005 | 无效的序列化类型 |
| 客户端 | `AIMRT_RPC_STATUS_CLI_SERIALIZATION_FAILED` | 2006 | 序列化失败 |
| 客户端 | `AIMRT_RPC_STATUS_CLI_INVALID_DESERIALIZATION_TYPE` | 2007 | 无效的反序列化类型 |
| 客户端 | `AIMRT_RPC_STATUS_CLI_DESERIALIZATION_FAILED` | 2008 | 反序列化失败 |
| 客户端 | `AIMRT_RPC_STATUS_CLI_NO_BACKEND_TO_HANDLE` | 2009 | 无后端处理 |
| 客户端 | `AIMRT_RPC_STATUS_CLI_SEND_REQ_FAILED` | 2010 | 请求发送失败 |

请注意，`Status`中的错误信息一般仅表示框架层面的错误，例如服务未找到、网络错误或者序列化错误等，供开发者排查框架层面的问题。如果开发者需要返回业务层面的错误，建议在业务包中添加相应的字段。

另外，使用 **ROS2 RPC 后端和 ROS2 Srv 结合**时，由于 ROS2 本身不支持返回除 request_id 和 response 之外的其他字段，所以框架侧不会返回服务端提供的错误码，而是直接返回一个 `AIMRT_RPC_STATUS_OK。`
例如，服务端某服务未实现，本应返回一个 `AIMRT_RPC_STATUS_SVR_NOT_IMPLEMENTED` 错误码，但是由于上述组合自身的限制，框架侧只会给客户端返回 `AIMRT_RPC_STATUS_OK。`

## Client

在 AimRT RPC 桩代码工具生成的代码里，如`rpc.aimrt_rpc.pb.h`或者`example.aimrt_rpc.srv.h`文件里，提供了四种类型的 Client Proxy 接口，开发者基于这些 Proxy 接口来发起 RPC 调用：
- **同步型接口**：名称一般为`XXXSyncProxy`；
- **异步回调型接口**：名称一般为`XXXAsyncProxy`；
- **异步 Future 型接口**：名称一般为`XXXFutureProxy`；
- **无栈协程型接口**：名称一般为`XXXCoProxy`；

这几种 Proxy 类型可以混合使用，开发者可以根据实际需求选择合适的类型。它们除了调用 RPC 时的 API 接口不一样，底层的运行表现是一致的。


### 公共接口

所有的 Proxy 都有一个公共基类，共享一些公共接口，如下所示：
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

具体说明如下：
- Proxy 一般都比较轻量，可以随用随创建；
- RPC Type 为固有属性，表示该 RPC Service 名称所属的体系，例如`pb`、`ros2`等。可以通过`RpcType`方法获取；
- RPC ServiceName 表示该 RPC 服务的名称，如果不做特殊配置，则会使用一个跟**协议名称**绑定的默认值。如果需要使用同一套协议来提供不同的服务，也可以通过`SetServiceName`方法进行设置；
- 所有类型的 Proxy 都需要从`aimrt::rpc::RpcHandleRef`句柄构造；
- 所有类型的 Proxy 提供了`RegisterClientFunc`静态方法，用于注册当前 RPC Client：
  - 该方法实际调用的是一个为当前 RPC 生成的`RegisterXXXClientFunc`全局方法；
  - 该方法需要传入`aimrt::rpc::RpcHandleRef`句柄作为参数；
  - 该方法可以选择传入一个 RPC ServiceName 字段，作为注册时的 RPC 服务名称；
- 如果有多个同类型的 Proxy，则通过 `ServiceName` 作为区分。开发者需要保证注册时和使用时的 `ServiceName` 一致；
- 可以为 Proxy 设置一个默认 Context：
  - 如果在调用 RPC 时未传入 Context 或者传入了空的 Context，则会使用该 Proxy 默认的 Context；
  - 使用者可以通过`SetDefaultContextSharedPtr`和`GetDefaultContextSharedPtr`方法来设置、获取默认 Context；
  - 使用者可以通过`NewContextSharedPtr`方法从默认 Context 复制得到一份新的 Context；

注意：开发者发起一个 RPC 调用后，特定的 RPC 后端将处理具体的请求，实际的耗时、性能等表现以及 timeout 功能等和运行时配置的后端有关，在开发阶段无法确定，详细信息请参考对应后端的文档。


### 同步型接口

同步型接口在使用上最简单，但在运行效率上是最低的。它通过阻塞当前线程，等待 RPC 接口返回。一般可以在一些不要求性能的场合为了提高开发效率而使用这种方式，但不推荐在高性能要求的场景使用。


使用同步型接口发起 RPC 调用非常简单，一般分为以下几个步骤：
- **Step 0**：引用桩代码头文件，例如`xxx.aimrt_rpc.pb.h`或者`xxx.aimrt_rpc.srv.h`，其中有同步接口的句柄`XXXSyncProxy`；
- **Step 1**：在`Initialize`阶段调用`RegisterClientFunc`方法注册 RPC Client；
- **Step 2**：在`Start`阶段里某个业务函数里发起 RPC 调用：
  - **Step 2-1**：创建一个`XXXSyncProxy`，构造参数是`aimrt::rpc::RpcHandleRef`类型句柄；
  - **Step 2-2**：创建 Req、Rsp，并填充 Req 内容；
  - **Step 2-3**：【可选】创建 ctx，设置超时等信息；
  - **Step 2-4**：基于 proxy，传入 ctx、Req、Rsp，发起 RPC 调用，同步等待 RPC 调用结束，保证在整个调用周期里 ctx、Req、Rsp 都保持有效且不会改动，最终获取返回的 status；
  - **Step 2-5**：解析 status 和 Rsp；


以下是一个简单的基于 protobuf 的示例，基于 ROS2 Srv 的语法也基本类似：
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


更多示例请参考：
- {{ '[pb_rpc_sync_client]({}/src/examples/cpp/pb_rpc/module/normal_rpc_sync_client_module/normal_rpc_sync_client_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_sync_client]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_sync_client_module/normal_rpc_sync_client_module.cc)'.format(code_site_root_path_url) }}


### 异步回调型接口

异步回调型接口使用回调来返回异步结果，在性能上表现最好，但开发友好度是最低的，很容易陷入回调地狱。

使用异步回调型接口发起 RPC 调用一般分为以下几个步骤：
- **Step 0**：引用桩代码头文件，例如`xxx.aimrt_rpc.pb.h`或者`xxx.aimrt_rpc.srv.h`，其中有异步接口的句柄`XXXAsyncProxy`；
- **Step 1**：在`Initialize`阶段调用`RegisterClientFunc`方法注册 RPC Client；
- **Step 2**：在`Start`阶段里某个业务函数里发起 RPC 调用：
  - **Step 2-1**：创建一个`XXXAsyncProxy`，构造参数是`aimrt::rpc::RpcHandleRef`；
  - **Step 2-2**：创建 Req、Rsp，并填充 Req 内容；
  - **Step 2-3**：【可选】创建 ctx，设置超时等信息；
  - **Step 2-4**：基于 proxy，传入 ctx、Req、Rsp 和结果回调，发起 RPC 调用，保证在整个调用周期里 ctx、Req、Rsp 都保持有效且不会改动；
  - **Step 2-5**：在回调函数中获取返回的 status，解析 status 和 Rsp；

前几个步骤与同步型接口基本一致，区别在于**Step 2-4**需要使用异步回调的方式来获取结果。以下是一个简单的基于 protobuf 的示例，基于 ROS2 Srv 的语法也基本类似：
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


更多示例请参考：
- {{ '[pb_rpc_async_client]({}/src/examples/cpp/pb_rpc/module/normal_rpc_async_client_module/normal_rpc_async_client_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_async_client]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_async_client_module/normal_rpc_async_client_module.cc)'.format(code_site_root_path_url) }}


### 异步Future型接口


异步 Future 型接口基于`std::future`来返回异步结果，开发者可以在发起 RPC 调用后先去做其他事情，等需要 RPC 结果时在调用`std::future::get`方法来阻塞的获取结果。它在一定程度上兼顾了性能和开发友好度，属于同步型和异步回调型中间的一个选择。


使用异步 Future 型接口发起 RPC 调用一般分为以下几个步骤：
- **Step 0**：引用桩代码头文件，例如`xxx.aimrt_rpc.pb.h`或者`xxx.aimrt_rpc.srv.h`，其中有异步接口的句柄`XXXFutureProxy`；
- **Step 1**：在`Initialize`阶段调用`RegisterClientFunc`方法注册 RPC Client；
- **Step 2**：在`Start`阶段里某个业务函数里发起 RPC 调用：
  - **Step 2-1**：创建一个`XXXFutureProxy`，构造参数是`aimrt::rpc::RpcHandleRef`；
  - **Step 2-2**：创建 Req、Rsp，并填充 Req 内容；
  - **Step 2-3**：【可选】创建 ctx，设置超时等信息；
  - **Step 2-4**：基于 proxy，传入 ctx、Req、Rsp 和结果回调，发起 RPC 调用，保证在整个调用周期里 ctx、Req、Rsp 都保持有效且不会改动，并获取一个`std::future<Status>`句柄；
  - **Step 2-5**：在后续某个时间，阻塞的调用`std::future<Status>`句柄的`get()`方法，获取 status 值，并解析 status 和 Rsp；



以下是一个简单的基于 protobuf 的示例，基于 ROS2 Srv 的语法也基本类似：
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


更多示例请参考：
- {{ '[pb_rpc_future_client]({}/src/examples/cpp/pb_rpc/module/normal_rpc_future_client_module/normal_rpc_future_client_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_future_client]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_future_client_module/normal_rpc_future_client_module.cc)'.format(code_site_root_path_url) }}


### 无栈协程型接口


AimRT 为 RPC Client 端提供了一套基于 C++20 协程和[C++ executors 提案](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p0443r14.html)当前的一个实现库[libunifex](https://github.com/facebookexperimental/libunifex)来实现的一套无栈协程形式的接口。无栈协程接口在本质上是对异步回调型接口的封装，在性能上基本与异步回调型接口一致，但大大提升了开发友好度。

使用协程型接口发起 RPC 调用一般分为以下几个步骤：
- **Step 0**：引用桩代码头文件，例如`xxx.aimrt_rpc.pb.h`或者`xxx.aimrt_rpc.srv.h`，其中有协程接口的句柄`XXXCoProxy`；
- **Step 1**：在`Initialize`阶段调用`RegisterClientFunc`方法注册 RPC Client；
- **Step 2**：在`Start`阶段里某个业务协程里发起 RPC 调用：
  - **Step 2-1**：创建一个`XXXCoProxy`，构造参数是`aimrt::rpc::RpcHandleRef`；
  - **Step 2-2**：创建 Req、Rsp，并填充 Req 内容；
  - **Step 2-3**：【可选】创建 ctx，设置超时等信息；
  - **Step 2-4**：基于 proxy，传入 ctx、Req、Rsp 和结果回调，发起 RPC 调用，在协程中等待 RPC 调用结束，保证在整个调用周期里 ctx、Req、Rsp 都保持有效且不会改动，获取返回的 status；
  - **Step 2-5**：解析 status 和 Rsp；

整个接口风格与同步型接口几乎一样，但必须要在协程中调用。以下是一个简单的基于 protobuf 的示例，基于 ROS2 Srv 的语法也基本类似：
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


更多示例请参考：
- {{ '[pb_rpc_co_client]({}/src/examples/cpp/pb_rpc/module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_co_client]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)'.format(code_site_root_path_url) }}


## Server

在 AimRT RPC 桩代码工具生成的代码里，如`rpc.aimrt_rpc.pb.h`或者`example.aimrt_rpc.srv.h`里，提供了三种类型的 Service 基类，开发者继承这些 Service 基类，实现其中的虚接口来提供实际的 RPC 服务：
- **同步型接口**：名称一般为`XXXSyncService`；
- **异步回调型接口**：名称一般为`XXXAsyncService`；
- **无栈协程型接口**：名称一般为`XXXCoService`；

在单个 service 内，这三种类型的不能混合使用，只能选择一种，开发者可以根据自身需求选用。


### 公共接口

所有的 Service 都有一个公共基类，共享一些公共接口，如下所示：
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

具体说明如下：
- 开发者需要继承这些 Service 基类，来实现业务逻辑。开发者需要自行管理业务 Service 实例的生命周期；
- RPC Type 为固有属性，表示该 RPC Service 名称所属的体系，例如`pb`、`ros2`等。可以通过`RpcType`方法获取；
- RPC ServiceName 表示该 RPC 服务的名称，如果不做特殊配置，则会使用一个跟**协议名称**绑定的默认值。如果需要使用同一套协议来提供不同的服务，也可以通过`SetServiceName`方法进行设置；

注意：由哪个执行器来执行 Service 回调，这和具体的 RPC 后端实现有关，在运行阶段通过配置才能确定，使用者在编写逻辑代码时不应有任何假设，详细信息请参考对应后端的文档。


最佳实践是：如果回调中的任务非常轻量，比如只是设置一个变量，那就可以直接在回调里处理；但如果回调中的任务比较重，那最好调度到其他专门执行任务的执行器里进行处理。


### 同步型接口

同步型接口在使用上最简单，但很多时候业务的 RPC 处理函数中需要继续请求下游，会有一些异步调用，这种情况下只能阻塞的等待下游调用完成，可能会造成运行效率上的降低。一般可以在处理一些简单的请求、不需要发起其他异步调用的场景下使用同步型接口。


使用同步型接口实现 RPC 服务，一般分为以下几个步骤：
- **Step 0**：引用桩代码头文件，例如`xxx.aimrt_rpc.pb.h`或者`xxx.aimrt_rpc.srv.h`，其中有同步接口的 Service 基类`XXXSyncService`；
- **Step 1**：开发者实现一个 Impl 类，继承`XXXSyncService`，并实现其中的虚接口；
  - **Step 1-1**：解析 Req，并填充 Rsp；
  - **Step 1-2**：返回`Status`；
- **Step 2**：在`Initialize`阶段调用`RpcHandleRef`的`RegisterService`方法注册 RPC Service；


以下是一个简单的基于 protobuf 的示例，基于 ROS2 Srv 的语法也基本类似：
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


更多示例请参考：
- {{ '[pb_rpc_sync_service]({}/src/examples/cpp/pb_rpc/module/normal_rpc_sync_server_module/service.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_sync_service]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_sync_server_module/service.cc)'.format(code_site_root_path_url) }}



### 异步回调型接口


异步回调型接口会传递一个回调给开发者，开发者在 RPC 处理完成后调用这个回调来传递最终处理结果。这种方式可以在 RPC 中发起其他异步调用，由于不会阻塞，因此性能表现通常最好，但通常会导致开发出的代码难以阅读和维护。


使用异步回调型接口实现 RPC 服务，一般分为以下几个步骤：
- **Step 0**：引用桩代码头文件，例如`xxx.aimrt_rpc.pb.h`或者`xxx.aimrt_rpc.srv.h`，其中有异步接口的 Service 基类`XXXAsyncService`；
- **Step 1**：开发者实现一个 Impl 类，继承`XXXAsyncService`，并实现其中的虚接口；
  - **Step 1-1**：解析 Req，并填充 Rsp；
  - **Step 1-2**：调用 callback 将`Status`传递回去；
- **Step 2**：在`Initialize`阶段调用`RpcHandleRef`的`RegisterService`方法注册 RPC Service；


以下是一个简单的基于protobuf的示例，基于ROS2 Srv的语法也基本类似：
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


更多示例请参考：
- {{ '[pb_rpc_async_service]({}/src/examples/cpp/pb_rpc/module/normal_rpc_async_server_module/service.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_async_service]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_async_server_module/service.cc)'.format(code_site_root_path_url) }}


### 无栈协程型接口


与 RPC Client 端一样，在 RPC Service 端，AimRT 也提供了一套基于 C++20 协程和[C++ executors 提案](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p0443r14.html)当前的一个实现库[libunifex](https://github.com/facebookexperimental/libunifex)来实现的一套无栈协程形式的接口。无栈协程接口在本质上是对异步回调型接口的封装，在性能上基本与异步回调型接口一致，但大大提升了开发友好度。


使用协程型接口实现 RPC 服务，一般分为以下几个步骤：
- **Step 0**：引用桩代码头文件，例如`xxx.aimrt_rpc.pb.h`或者`xxx.aimrt_rpc.srv.h`，其中有协程接口的 Service 基类`XXXCoService`；
- **Step 1**：开发者实现一个 Impl 类，继承`XXXCoService`，并实现其中的虚接口；
  - **Step 1-1**：解析 Req，并填充 Rsp；
  - **Step 1-2**：使用 co_return 返回`Status`；
- **Step 2**：在`Initialize`阶段调用`RpcHandleRef`的`RegisterService`方法注册 RPC Service；


整个接口风格与同步型接口几乎一样。以下是一个简单的基于 protobuf 的示例，基于 ROS2 Srv 的语法也基本类似：
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

更多示例请参考：
- {{ '[pb_rpc_co_service]({}/src/examples/cpp/pb_rpc/module/normal_rpc_co_server_module/service.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_rpc_co_service]({}/src/examples/cpp/ros2_rpc/module/normal_rpc_co_server_module/service.cc)'.format(code_site_root_path_url) }}

## Context

开发者在调用 RPC 时，可以传入一个`aimrt::rpc::Context`，在处理 RPC 时，也会得到一个`aimrt::rpc::ContextRef`。`ContextRef`类型是`Context`类型的引用，两者包含的接口基本一致，它们最主要的功能是携带 Timeout 配置和一些 Key-Val 数据，用于向下游或 RPC 后端传递特定的信息。


其接口如下所示：

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

使用`Context`或`ContextRef`类型的 RPC ctx 时需要注意：
- RPC ctx 分为 Client 端和 Server 端两种类型，在构造时确定，无法修改，分别用于 Client 和 Server 场景；
- 可以使用`SetTimeout`、`Timeout`方法来设置、获取 ctx 中超时配置；
- 可以使用`SetMetaValue`、`GetMetaValue`方法来设置、获取 ctx 中的 Key-Val 值，使用`GetMetaKeys`来获取当前所有的 Key 值；

AimRT 在{{ '[rpc_context_base.h]({}/src/interface/aimrt_module_c_interface/rpc/rpc_context_base.h)'.format(code_site_root_path_url) }}文件中定义了一些特殊的 Key，业务使用这些特殊 Key 时应遵循一定的规则，这些特殊的 Key 包括：
- **AIMRT_RPC_CONTEXT_KEY_TO_ADDR**：用于设置 RPC 的对端地址，对端地址应遵循标准 URL 格式：`{{protocol}}://{{path}}`，其中`{{protocol}}`字段将用于 AimRT 框架选择 RPC 后端，`{{path}}`字段可用于不同后端的自定义行为；
- **AIMRT_RPC_CONTEXT_KEY_SERIALIZATION_TYPE**：用于设置消息的序列化类型，必须是注册时 type support 中支持的类型；
- **AIMRT_RPC_CONTEXT_KEY_FUNCTION_NAME**：用于传递 RPC Function 名称；
- **AIMRT_RPC_CONTEXT_KEY_BACKEND**：用于给 Server 端传递实际处理的后端名称；


在 Client 端，`Context`主要是用于在调用 RPC 时传入一些特殊的信息给 AimRT 框架和 RPC 后端，其使用时需要注意以下几点：
- 开发者可以直接构造一个`Context`类型实例，并自行负责其生命周期；
- 只能给 Client 端的 RPC 调用方法端传入 Client 类型的 ctx；
- 每个 `Context` 只能用于一次 Client 端调用，在传递给 Client 端的 RPC 调用方法后，状态即会被置为`Used`，如果未经`Reset`就用于下一次 RPC 调用，请求将出错；
- Client 端的 RPC 调用方法实际接受的是`ContextRef`类型作为参数，但`Context`类型可以隐式的转换为`ContextRef`类型；
- 开发者可以向 ctx 中设置 Timeout，但对 Timeout 的处理方式取决于实际的 RPC 后端，具体的处理方式请参考特定 RPC 后端的文档。
- 开发者可以向 ctx 中设置一些信息传递给具体的 RPC 后端，不同的后端对于 ctx 中的信息会有不同的处理方式，有的会读取其中特定的 Key-Val 值来特化传输行为，有的会将所有 Key-Val 信息透传到下游，具体的处理方式请参考特定 RPC 后端的文档。



在 Server 端，开发者可以在回调处理函数中接收`ContextRef`类型的参数，其使用时需要注意以下几点：
- 传递给回调处理函数的 ctx 生命周期由 AimRT 框架管理，与 Req、Rsp 的生命周期一致；
- 传递给回调处理函数的 ctx 是 Server 类型的，并且是`Used`状态；
- 传递给回调处理函数的 ctx 中可能会有 Timeout 信息和一些 Key-Val 信息，具体会传递哪些信息则由实际的 RPC 后端决定，请参考特定 RPC 后端的文档。



此外，一般来说在一个复杂业务系统中，一些服务端会在收到请求后，向更下游的服务再发起请求，从而形成逻辑层面上的长链路。如果要在框架层面打通这条逻辑上的链路，来实现一些监控、调度上的功能，就需要将 Server 类型的 ctx 中的特定信息同步到 Client 类型的 ctx 中，有两种方式：


1. 可以使用`RpcHandleRef`类型提供的`MergeServerContextToClientContext`方法，例如：
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

2. 可以使用 Proxy 的`NewContextSharedPtr`方法，将 Server 类型的 ctx 作为参数传递给该方法，例如：
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




