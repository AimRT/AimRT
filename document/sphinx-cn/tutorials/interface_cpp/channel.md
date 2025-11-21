# Channel

## 相关链接

代码文件：
- {{ '[aimrt_module_cpp_interface/channel/channel_context.h]({}/src/interface/aimrt_module_cpp_interface/channel/channel_context.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/channel/channel_handle.h]({}/src/interface/aimrt_module_cpp_interface/channel/channel_handle.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_protobuf_interface/channel/protobuf_channel.h]({}/src/interface/aimrt_module_protobuf_interface/channel/protobuf_channel.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_ros2_interface/channel/ros2_channel.h]({}/src/interface/aimrt_module_ros2_interface/channel/ros2_channel.h)'.format(code_site_root_path_url) }}

参考示例：
- {{ '[pb_chn]({}/src/examples/cpp/pb_chn)'.format(code_site_root_path_url) }}
  - {{ '[normal_publisher_module.cc]({}/src/examples/cpp/pb_chn/module/normal_publisher_module/normal_publisher_module.cc)'.format(code_site_root_path_url) }}
  - {{ '[normal_subscriber_module.cc]({}/src/examples/cpp/pb_chn/module/normal_subscriber_module/normal_subscriber_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_chn]({}/src/examples/cpp/ros2_chn)'.format(code_site_root_path_url) }}
  - {{ '[normal_publisher_module.cc]({}/src/examples/cpp/ros2_chn/module/normal_publisher_module/normal_publisher_module.cc)'.format(code_site_root_path_url) }}
  - {{ '[normal_subscriber_module.cc]({}/src/examples/cpp/ros2_chn/module/normal_subscriber_module/normal_subscriber_module.cc)'.format(code_site_root_path_url) }}

## 协议

协议用于确定通信各端的消息格式。一般来说，协议都是使用一种与具体的编程语言无关的 IDL ( Interface description language )描述，然后由某种工具转换为各个语言的代码。此处简要介绍一下 AimRT 官方支持的两种 IDL 如何转换为 Cpp 代码，进阶的使用方式请参考对应 IDL 的官方文档。

### Protobuf

[Protobuf](https://protobuf.dev/)是一种由 Google 开发的、用于序列化结构化数据的轻量级、高效的数据交换格式，是一种广泛使用的 IDL。


在使用时，开发者需要先定义一个`.proto`文件，在其中定义一个消息结构。例如`example.proto`：

```protobuf
syntax = "proto3";

message ExampleMsg {
  string msg = 1;
  int32 num = 2;
}
```

然后使用 Protobuf 官方提供的 protoc 工具进行转换，生成 C++ 代码，例如：
```shell
protoc --cpp_out=. example.proto
```

这将生成`example.pb.h`和`example.pb.cc`文件，包含了根据定义的消息类型生成的 C++ 类和方法。

请注意，以上这套原生的代码生成方式只是为了给开发者展示底层的原理，实际使用时还需要手动处理依赖和 CMake 封装等方面的问题，比较繁琐。AimRT 对这个过程进行了一定的封装，开发者可以直接使用{{ '[ProtobufGenCode.cmake]({}/cmake/ProtobufGenCode.cmake)'.format(code_site_root_path_url) }}文件中提供的两个 CMake 方法：
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


使用示例如下：
```cmake
# Generate C++ code for all '.proto' files in the current folder
add_protobuf_gencode_target_for_proto_path(
  TARGET_NAME example_pb_gencode
  PROTO_PATH ${CMAKE_CURRENT_SOURCE_DIR}
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR})
```

之后只要链接`example_pb_gencode`这个 CMake Target 即可使用该协议。例如：
```cmake
target_link_libraries(my_lib PUBLIC example_pb_gencode)
```

### ROS2 Message

ROS2 Message 是一种用于在 ROS2 中进行通信和数据交换的结构化数据格式。在使用时，开发者需要先定义一个 ROS2 Package，在其中定义一个`.msg`文件，比如`example.msg`：

```
int32   num
float32 num2
char    data
```

然后直接通过 ROS2 提供的 CMake 方法`rosidl_generate_interfaces`，为消息生成 C++ 代码和 CMake Target，例如：
```cmake
rosidl_generate_interfaces(
  example_msg_gencode
  "msg/example.msg"
)
```

在这之后就可以引用相关的 CMake Target 来使用生成的 C++ 代码。详情请参考 ROS2 的官方文档和 AimRT 提供的 Example。


## ChannelHandle

AimRT 中，模块可以通过调用`CoreRef`句柄的`GetChannelHandle()`接口，获取`aimrt::channel::ChannelHandleRef`句柄，来使用 Channel 功能。其提供的核心接口如下：
```cpp
namespace aimrt::channel {

class ChannelHandleRef {
 public:
  PublisherRef GetPublisher(std::string_view topic) const;

  SubscriberRef GetSubscriber(std::string_view topic) const;

  void MergeSubscribeContextToPublishContext(
    const ContextRef subscribe_ctx_ref, ContextRef publish_ctx_ref) const;
};

}  // namespace aimrt::channel
```


开发者可以调用`ChannelHandleRef`中的`GetPublisher`方法和`GetSubscriber`方法，获取指定 Topic 名称的`PublisherRef`和`SubscriberRef`类型句柄，分别用于 Channel 发布和订阅。这两个方法使用注意如下：
  - 这两个接口是线程安全的。
  - 这两个接口可以在`Initialize`阶段和`Start`阶段使用。


`PublisherRef`和`SubscriberRef`句柄提供了一个与具体协议类型无关的 Api 接口，但除非开发者想要使用自定义的消息类型，才需要直接调用它们提供的接口。

AimRT 官方支持了两种协议类型：**Protobuf** 和 **Ros2 Message**，并提供了这两种协议类型的 Channel 接口封装。这两套 Channel 接口除了协议类型不同，整体的 Api 风格都一致，开发者一般直接使用这两套与协议类型绑定的 Channel 接口即可。使用时需要分别引用对应的 CMake Target：
- Protobuf Channel：需 CMake 引用 `aimrt::interface::aimrt_module_protobuf_interface`；
- Ros2 Channel：需 CMake 引用 `aimrt::interface::aimrt_module_ros2_interface`；


开发者还可以使用`MergeSubscribeContextToPublishContext`方法，来将 subscribe 端的 context 信息传递到 publish 端的 context 中，可以用于打通整条数据链路。详情请参考 Context 章节的说明。


## Publish

AimRT 提供了**函数风格**和**Proxy风格**两种风格的接口来发布一个消息：

- 函数风格接口：
```cpp
namespace aimrt::channel {

template <typename MsgType>
bool RegisterPublishType(PublisherRef publisher);

template <typename MsgType>
void Publish(PublisherRef publisher, aimrt::channel::ContextRef ctx_ref, const MsgType& msg);

template <typename MsgType>
void Publish(PublisherRef publisher, const MsgType& msg);

}  // namespace aimrt::channel
```

- Proxy 类风格接口：
```cpp
namespace aimrt::channel {

template <typename MsgType>
class PublisherProxy {
 public:
  explicit PublisherProxy(PublisherRef publisher);

  // Context
  std::shared_ptr<Context> NewContextSharedPtr(ContextRef ctx_ref = ContextRef()) const;
  void SetDefaultContextSharedPtr(const std::shared_ptr<Context>& ctx_ptr);
  std::shared_ptr<Context> GetDefaultContextSharedPtr() const;

  // Register type
  static bool RegisterPublishType(PublisherRef publisher);
  bool RegisterPublishType() const;

  // Publish
  void Publish(ContextRef ctx_ref, const MsgType& msg) const;
  void Publish(const MsgType& msg) const;
};

}  // namespace aimrt::channel
```

Proxy 类型接口可以绑定类型信息和一个默认 Context，功能更齐全一些。但两种风格接口的基本使用效果是一致的，用户需要两个步骤来实现逻辑层面的消息发布：
- **Step 1**：使用`RegisterPublishType`方法注册消息类型：
  - 只能在`Initialize`阶段注册；
  - 不允许在一个`PublisherRef`中重复注册同一种类型；
  - 如果注册失败，会返回 false；
- **Step 2**：使用`Publish`方法发布数据：
  - 只能在`Start`阶段之后发布数据；
  - 有两种`Publish`接口，其中一种多一个 Context 参数，用于向后端、下游传递一些额外信息，Context 的详细说明见后续章节；
  - 在调用`Publish`接口时，开发者应保证传入的 Context 和 Msg 在`Publish`接口返回之前都不会发生变化，否则行为是未定义的；


用户`Publish`一个消息后，特定的 Channel 后端将处理具体的消息发布请求。此时根据不同后端的实现，有可能会阻塞一段时间，因此`Publish`方法耗费的时间是未定义的。但一般来说，Channel 后端都不会阻塞`Publish`方法太久，详细信息请参考对应后端的文档。


## Subscribe

与发布接口一样，AimRT 提供了**函数风格**和**Proxy风格**两种风格类型的接口来订阅一个消息，同时还提供了**智能指针形式**和**协程形式**两种回调函数：

- 函数风格接口：
```cpp
// Callback accept a CTX and a smart pointer as parameters
template <MsgType>
bool Subscribe(
    SubscriberRef subscriber,
    std::function<void(ContextRef, const std::shared_ptr<const MsgType>&)>&& callback);

// Callback accept a pointer as a parameter
template <MsgType>
bool Subscribe(
    SubscriberRef subscriber,
    std::function<void(const std::shared_ptr<const MsgType>&)>&& callback);

// Coroutine callback, accept a CTX and a const reference to message as parameters
template <MsgType>
bool SubscribeCo(
    SubscriberRef subscriber,
    std::function<co::Task<void>(ContextRef, const MsgType&)>&& callback);

// Coroutine callback, accept a const reference to message as a parameter
template <MsgType>
bool SubscribeCo(
    SubscriberRef subscriber,
    std::function<co::Task<void>(const MsgType&)>&& callback);
```

- Proxy 类风格接口：
```cpp
namespace aimrt::channel {

template <typename MsgType>
class SubscriberProxy {
 public:
  explicit SubscriberProxy(SubscriberRef subscriber);

  // Callback accept a CTX and a smart pointer as parameters
  bool Subscribe(
      std::function<void(ContextRef, const std::shared_ptr<const MsgType>&)>&& callback) const;

  // Callback accept a pointer as a parameter
  bool Subscribe(
      std::function<void(const std::shared_ptr<const MsgType>&)>&& callback) const;

  // Coroutine callback, accept a CTX and a const reference to message as parameters
  bool SubscribeCo(
      std::function<co::Task<void>(ContextRef, const MsgType&)>&& callback) const;

  // Coroutine callback, accept a const reference to message as a parameter
  bool SubscribeCo(std::function<co::Task<void>(const MsgType&)>&& callback) const;
};

}  // namespace aimrt::channel
```

Proxy 类型接口可以绑定类型信息，功能更齐全一些。但两种风格接口的基本使用效果是一致的，使用 Subscribe 接口时需要注意：
- 只能在`Initialize`阶段调用订阅接口；
- 不允许在一个`SubscriberRef`中重复订阅同一种类型；
- 如果订阅失败，会返回 false；
- 可以传入两种回调函数，其中一种多一个 Context 参数，用于向传递一些额外信息，Context 的详细说明见后续章节；
- Context 和 Msg 的生命周期：
  - 对于接收智能指针形式 Msg 的回调，Context 和 Msg 的生命周期将持续到 Msg 的智能指针引用计数归零析构时；
  - 对于协程形式的回调，Context 和 Msg 的生命周期将持续到协程退出为止；


此外还需要注意的是，由哪个执行器来执行订阅的回调，这和具体的 Channel 后端实现有关，在运行阶段通过配置才能确定，使用者在编写逻辑代码时不应有任何假设，详细信息请参考对应后端的文档。


最佳实践是：如果回调中的任务非常轻量，比如只是设置一个变量，那就可以直接在回调里处理；但如果回调中的任务比较重，那最好调度到其他专门执行任务的执行器里进行处理。


## Context

开发者在发布 Channel 消息时，可以传入一个`aimrt::channel::Context`，在订阅 Channel 消息时，也可以选择在回调中接收一个`aimrt::channel::ContextRef`。`ContextRef`类型是`Context`类型的引用，两者包含的接口基本一致，它们最主要的功能是携带一些 Key-Val 数据，用于向下游或 Channel 后端传递特定的信息。


其接口如下所示：

```cpp
namespace aimrt::channel {

class Context {
 public:
  bool CheckUsed() const;
  void SetUsed();
  void Reset();

  aimrt_channel_context_type_t GetType() const;

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
  explicit ContextRef(const aimrt_channel_context_base_t* base_ptr);

  bool CheckUsed() const;
  void SetUsed();
  void Reset();

  aimrt_channel_context_type_t GetType() const;

  std::string_view GetMetaValue(std::string_view key) const;
  void SetMetaValue(std::string_view key, std::string_view val);
  std::vector<std::string_view> GetMetaKeys() const;

  std::string ToString() const;
};

}  // namespace aimrt::channel
```


使用`Context`或`ContextRef`类型的 Channel ctx 时需要注意：
- Channel ctx 分为 Publish 端和 Subscribe 端两种类型，在构造时确定，无法修改，分别用于 Publish 和 Subscribe 场景；
- 可以使用`SetMetaValue`、`GetMetaValue`方法来设置、获取 ctx 中的 Key-Val 值，使用`GetMetaKeys`来获取当前所有的 Key 值；


AimRT 在{{ '[channel_context_base.h]({}/src/interface/aimrt_module_c_interface/channel/channel_context_base.h)'.format(code_site_root_path_url) }}文件中定义了一些特殊的 Key，业务使用这些特殊 Key 时应遵循一定的规则，这些特殊的 Key 包括：
- **AIMRT_CHANNEL_CONTEXT_KEY_SERIALIZATION_TYPE**：用于设置消息的序列化类型，必须是注册时 type support 中支持的类型；
- **AIMRT_CHANNEL_CONTEXT_KEY_BACKEND**：用于给 Subscribe 端传递实际处理的后端名称；
- **AIMRT_CHANNEL_CONTEXT_KEY_TO_ADDR**：用于给 Publish 端传递实际处理的服务端地址，格式为：`backend://ip:port;backend://ip:port;...`，其中`backend`为后端类型，`ip`和`port`为实际处理的服务端地址，目前支持`http`、`tcp`、`udp`三种后端类型，当其中包含某个后端的地址时，将不再使用配置文件中指定的地址进行发送，例如指定为 `http://127.0.0.1:50090;tcp://127.0.0.1:50060`，则配置文件中指定的 http 和 tcp 地址将不会被使用，仅向这两个地址进行发送。
- **AIMRT_CHANNEL_CONTEXT_KEY_PUB_SEQ**：用于在 Publish / Subscribe 场景中传递消息的发布序列号。该值在同一 Topic 内由框架自动维护为单调递增（单发布者场景下可认为唯一），业务方可以基于该字段进行丢包、乱序排查等分析。
- **AIMRT_CHANNEL_CONTEXT_KEY_PUB_TIMESTAMP**：用于在 Publish / Subscribe 场景中传递消息的发布时间戳，单位为纳秒。一般由框架在发布时自动填充为当前时间（或由上游中间件透传其源时间戳，例如 ROS 2 的 `source_timestamp`），部分插件会使用该字段写入文件（如 MCAP 的 `publishTime` 字段），业务方可用于计算端到端时延等。

在 Publish 端，`Context`主要是用于在调用`Publish`方法时传入一些特殊的信息给 AimRT 框架和 Channel 后端，其使用时需要注意以下几点：
- 开发者可以直接构造一个`Context`类型实例，并自行负责其生命周期；
- 只能给`Publish`方法传入 Publish 类型的 ctx；
- 每个 `Context` 只能用于一次 Publish 过程，在传递给`Publish`方法后，状态即会被置为`Used`，如果未经`Reset`就用于下一次 Publish，消息将不会被正确发布；
- `Publish`方法实际接受的是`ContextRef`类型作为参数，但`Context`类型可以隐式的转换为`ContextRef`类型；
- 开发者可以向 ctx 中设置一些信息传递给具体的 Channel 后端，不同的后端对于 ctx 中的信息会有不同的处理方式，有的会读取其中特定的 Key-Val 值来特化传输行为，有的会将所有 Key-Val 信息透传到下游，具体的处理方式请参考特定 Channel 后端的文档。



在 Subscribe 端，开发者可以选择在回调处理函数中接收`ContextRef`类型的参数，其使用时需要注意以下几点：
- 传递给回调处理函数的 ctx 生命周期由 AimRT 框架管理，与 Msg 的生命周期一致；
- 传递给回调处理函数的 ctx 是 Subscribe 类型的，并且是`Used`状态；
- 传递给回调处理函数的 ctx 中可能会有一些 Key-Val 信息，具体会传递哪些信息则由 Channel 后端决定，请参考特定 Channel 后端的文档。



此外，一般来说在一个复杂业务系统中，一些订阅者会在收到消息后发布新的消息到更下游，会存在很多条逻辑层面上的长链路。如果要在框架层面打通这条逻辑上的链路，来实现一些监控、调度上的功能，就需要将 Subscribe 类型的 ctx 中的特定信息同步到 Publish 类型的 ctx 中，有两种方式：

1. 可以使用`PublisherRef`或`ChannelHandleRef`类型提供的`MergeSubscribeContextToPublishContext`方法，例如：
```cpp
aimrt::channel::PublisherRef publisher;

// Subscribe callback
void EventHandle(ContextRef subscribe_ctx, const std::shared_ptr<const FooMsg>& msg) {
    BarMsg new_msg;

    Context publishe_ctx;
    publisher.MergeSubscribeContextToPublishContext(subscribe_ctx, publishe_ctx);

    aimrt::channel::Publish(publisher, publishe_ctx, new_msg);
}
```

2. 可以使用`aimrt::channel::PublisherProxy`的`NewContextSharedPtr`方法，将 Subscribe 类型的 ctx 作为参数传递给该方法，例如：
```cpp
aimrt::channel::PublisherProxy<BarMsg> publisher_proxy;

// Subscribe callback
void EventHandle(ContextRef subscribe_ctx, const std::shared_ptr<const FooMsg>& msg) {
    BarMsg new_msg;

    auto publishe_ctx = publisher_proxy.NewContextSharedPtr(subscribe_ctx);

    publisher_proxy.Publish(publishe_ctx, new_msg);
}
```


## 使用示例


以下是一个简单的发布消息的示例，基于 proxy 风格接口：
```cpp
#include "event.pb.h"

class NormalPublisherModule : public aimrt::ModuleBase {
 public:
  bool Initialize(aimrt::CoreRef core) override {
    core_ = core;

    // Register publish type
    std::string topic_name = "test_topic";
    publisher_ = core_.GetChannelHandle().GetPublisher(topic_name);
    aimrt::channel::RegisterPublishType<ExampleEventMsg>(publisher_);

    return true;
  }

  bool Start() override {
    // create publish proxy
    aimrt::channel::PublisherProxy<ExampleEventMsg> publisher_proxy(publisher_);

    // set msg
    ExampleEventMsg msg;
    msg.set_msg("hello world");

    // publish msg
    publisher_proxy.Publish(msg);
  }

  // ...

 private:
  aimrt::CoreRef core_;
  aimrt::channel::PublisherRef publisher_;
};
```


以下是一个简单的订阅消息的示例：
```cpp
#include "event.pb.h"

class NormalSubscriberModule : public aimrt::ModuleBase {
 public:
  bool Initialize(aimrt::CoreRef core) override {
    // Subscribe
    std::string topic_name = "test_topic";
    auto subscriber = core_.GetChannelHandle().GetSubscriber(topic_name);

    aimrt::channel::Subscribe<ExampleEventMsg>(
        subscriber,
        [](const std::shared_ptr<const ExampleEventMsg>& data) {
          // Handle msg ...
        });
  }

  // ...
};
```

