

# Channel

## Related Links

Code Files:
- {{ '[aimrt_module_cpp_interface/channel/channel_context.h]({}/src/interface/aimrt_module_cpp_interface/channel/channel_context.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/channel/channel_handle.h]({}/src/interface/aimrt_module_cpp_interface/channel/channel_handle.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_protobuf_interface/channel/protobuf_channel.h]({}/src/interface/aimrt_module_protobuf_interface/channel/protobuf_channel.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_ros2_interface/channel/ros2_channel.h]({}/src/interface/aimrt_module_ros2_interface/channel/ros2_channel.h)'.format(code_site_root_path_url) }}

Reference Examples:
- {{ '[pb_chn]({}/src/examples/cpp/pb_chn)'.format(code_site_root_path_url) }}
  - {{ '[normal_publisher_module.cc]({}/src/examples/cpp/pb_chn/module/normal_publisher_module/normal_publisher_module.cc)'.format(code_site_root_path_url) }}
  - {{ '[normal_subscriber_module.cc]({}/src/examples/cpp/pb_chn/module/normal_subscriber_module/normal_subscriber_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_chn]({}/src/examples/cpp/ros2_chn)'.format(code_site_root_path_url) }}
  - {{ '[normal_publisher_module.cc]({}/src/examples/cpp/ros2_chn/module/normal_publisher_module/normal_publisher_module.cc)'.format(code_site_root_path_url) }}
  - {{ '[normal_subscriber_module.cc]({}/src/examples/cpp/ros2_chn/module/normal_subscriber_module/normal_subscriber_module.cc)'.format(code_site_root_path_url) }}

## Protocol

Protocols define message formats for communication between endpoints. Typically, protocols are described using an IDL (Interface Description Language) that is programming language-agnostic, then converted to code for various languages using specific tools. This section briefly introduces how two supported IDLs in AimRT are converted to C++ code. For advanced usage, please refer to official documentation of corresponding IDLs.

### Protobuf

[Protobuf](https://protobuf.dev/) is a lightweight, efficient data interchange format developed by Google for serializing structured data, widely used as an IDL.

When using Protobuf, developers first need to define a `.proto` file containing message structures. Example `example.proto`:

```protobuf
syntax = "proto3";

message ExampleMsg {
  string msg = 1;
  int32 num = 2;
}
```

Then use the official protoc tool to generate C++ code:
```shell
protoc --cpp_out=. example.proto
```

This generates `example.pb.h` and `example.pb.cc` files containing C++ classes and methods corresponding to the defined message types.

Note that this native code generation approach demonstrates underlying principles. Actual usage requires manual handling of dependencies and CMake packaging, which can be cumbersome. AimRT provides encapsulation through {{ '[ProtobufGenCode.cmake]({}/cmake/ProtobufGenCode.cmake)'.format(code_site_root_path_url) }} with two CMake methods:
- `add_protobuf_gencode_target_for_proto_path`: Generates C++ code for `.proto` files in specified path
  - **TARGET_NAME**: Generated CMake target name
  - **PROTO_PATH**: Protocol file directory
  - **GENCODE_PATH**: Generated code output path
  - **DEP_PROTO_TARGETS**: Dependent Proto CMake targets
  - **OPTIONS**: Additional protoc parameters
- `add_protobuf_gencode_target_for_one_proto_file`: Generates C++ code for single `.proto` file
  - **TARGET_NAME**: Generated CMake target name
  - **PROTO_FILE**: Single protocol file path
  - **GENCODE_PATH**: Generated code output path
  - **DEP_PROTO_TARGETS**: Dependent Proto CMake targets
  - **OPTIONS**: Additional protoc parameters

Usage example:
```cmake
# Generate C++ code for all '.proto' files in the current folder
add_protobuf_gencode_target_for_proto_path(
  TARGET_NAME example_pb_gencode
  PROTO_PATH ${CMAKE_CURRENT_SOURCE_DIR}
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR})
```

Simply link the `example_pb_gencode` CMake target to use the protocol. Example:
```cmake
target_link_libraries(my_lib PUBLIC example_pb_gencode)
```

### ROS2 Message

ROS2 Message defines structured data formats for ROS2 communication. Developers need to create a ROS2 package containing `.msg` files, e.g., `example.msg`:

```
int32   num
float32 num2
char    data
```

Use ROS2's CMake method `rosidl_generate_interfaces` to generate C++ code and CMake targets:
```cmake
rosidl_generate_interfaces(
  example_msg_gencode
  "msg/example.msg"
)
```

Reference these CMake targets to use generated code. Details refer to ROS2 official documentation and AimRT examples.

## ChannelHandle

In AimRT, modules can obtain the `aimrt::channel::ChannelHandleRef` handle by calling the `GetChannelHandle()` interface of the `CoreRef` handle to use Channel functionality. Its core interfaces are as follows:
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

Developers can call the `GetPublisher` and `GetSubscriber` methods in `ChannelHandleRef` to obtain `PublisherRef` and `SubscriberRef` type handles for specified Topic names, used for Channel publishing and subscribing respectively. Notes for using these two methods:
  - These interfaces are thread-safe.
  - These interfaces can be used during both `Initialize` and `Start` phases.

The `PublisherRef` and `SubscriberRef` handles provide a protocol-type-agnostic API interface. However, developers only need to directly call their provided interfaces when wanting to use custom message types.

AimRT officially supports two protocol types: **Protobuf** and **Ros2 Message**, and provides Channel interface encapsulation for these two protocol types. Apart from different protocol types, these two sets of Channel interfaces maintain consistent API styles. Developers can directly use these protocol-bound Channel interfaces. Corresponding CMake targets should be referenced when using:
- Protobuf Channel: Requires CMake reference to `aimrt::interface::aimrt_module_protobuf_interface`
- Ros2 Channel: Requires CMake reference to `aimrt::interface::aimrt_module_ros2_interface`

Developers can also use the `MergeSubscribeContextToPublishContext` method to transmit context information from the subscribe end to the publish end, which can be used to connect the entire data pipeline. For details, please refer to the Context chapter.

## Publish

AimRT provides two style interfaces for message publishing: **Function-style** and **Proxy-style**:

- Function-style interface:
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

- Proxy-class style interface:
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

The Proxy-type interface can bind type information and a default Context, providing more comprehensive functionality. However, the basic publishing effects of both style interfaces are consistent. Users need two steps to implement logical-level message publishing:
- **Step 1**: Use the `RegisterPublishType` method to register message types:
  - Can only be registered during the `Initialize` phase
  - Repeated registration of the same type in a `PublisherRef` is prohibited
  - Returns false if registration fails
- **Step 2**: Use the `Publish` method to publish data:
  - Can only publish data after the `Start` phase
  - Provides two `Publish` interfaces, one with an additional Context parameter for transmitting extra information to backend/downstream (see Context chapter for details)
  - Developers must ensure the passed Context and Msg remain unchanged before the `Publish` interface returns, otherwise behavior is undefined

After users `Publish` a message, specific Channel backends will handle the concrete message publishing requests. Depending on different backend implementations, there might be blocking for some period, therefore the time consumption of the `Publish` method is undefined. Generally speaking, Channel backends won't block the `Publish` method for too long. For detailed information, please refer to corresponding backend documentation.

## Subscribe

Like the publishing interfaces, AimRT provides **function-style** and **Proxy-style** interfaces for message subscription, along with two callback forms: **smart pointer form** and **coroutine form**:

- Function-style interface:
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

- Proxy class-style interface:
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

The Proxy-style interfaces can bind type information with more comprehensive functionality. However, both interface styles share the same basic usage effects. When using Subscribe interfaces, note that:
- Subscription interfaces can only be called during the `Initialize` phase;
- Repeated subscriptions to the same type within a `SubscriberRef` are prohibited;
- Returns false if subscription fails;
- Two callback forms can be provided, one with an additional Context parameter for passing extra information (detailed Context explanation follows in subsequent sections);
- Lifespan of Context and Msg:
  - For callbacks receiving Msg in smart pointer form: Context and Msg persist until the Msg's smart pointer reference count reaches zero and destructs;
  - For coroutine-form callbacks: Context and Msg persist until the coroutine exits;

Additional important notes:
- The executor responsible for executing subscription callbacks depends on the specific Channel backend implementation, which is determined at runtime through configuration. Developers should make no assumptions when writing logic code. Refer to backend-specific documentation for details.

Best practices:
- If the callback task is lightweight (e.g., simply setting a variable), handle it directly in the callback;
- If the callback task is heavy, schedule it to other dedicated executors for processing;

## Context

When publishing Channel messages, developers can pass an `aimrt::channel::Context`. When subscribing to Channel messages, they can also choose to receive an `aimrt::channel::ContextRef` in the callback. The `ContextRef` type is a reference to the `Context` type, both containing essentially identical interfaces. Their primary function is to carry key-value data for transmitting specific information to downstream components or Channel backends.

The interface is defined as follows:

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

When using Channel ctx of `Context` or `ContextRef` types, note:
- Channel ctx has two types determined during construction: Publish-type and Subscribe-type, which cannot be modified and are used in publishing/subscribing scenarios respectively
- Use `SetMetaValue` and `GetMetaValue` methods to set/retrieve key-value pairs in ctx, and `GetMetaKeys` to obtain all current keys

AimRT defines some special keys in {{ '[channel_context_base.h]({}/src/interface/aimrt_module_c_interface/channel/channel_context_base.h)'.format(code_site_root_path_url) }}. Follow specific rules when using these special keys:
- **AIMRT_CHANNEL_CONTEXT_KEY_SERIALIZATION_TYPE**: Used to set message serialization type, must be a type supported by registered type support
- **AIMRT_CHANNEL_CONTEXT_KEY_BACKEND**: Used to pass actual backend name to Subscribe-side
- **AIMRT_CHANNEL_CONTEXT_KEY_TO_ADDR**: Used to pass server addresses to Publish-side in format `backend://ip:port;backend://ip:port;...`, where `backend` represents backend type (currently supports `http`, `tcp`, `udp`). When specified, configuration file addresses will be overridden. Example: `http://127.0.0.1:50090;tcp://127.0.0.1:50060` will disable http/tcp addresses from config

For Publish-side usage:
- Developers can directly construct `Context` instances and manage their lifecycle
- Can only pass Publish-type ctx to `Publish` method
- Each `Context` can only be used once for publishing. After passing to `Publish`, its state becomes `Used` - reuse without `Reset` will cause publishing failure
- `Publish` method actually accepts `ContextRef` parameter (implicit conversion from `Context` supported)
- Developers can set backend-specific information in ctx. Different backends may read specific keys or transparently transmit all key-values - refer to backend-specific documentation

For Subscribe-side usage:
- Callback-received ctx has lifecycle managed by AimRT framework, synchronized with Msg lifecycle
- Callback ctx is Subscribe-type and in `Used` state
- Key-values in callback ctx depend on Channel backend implementation - refer to backend documentation

For cross-context synchronization in complex systems (monitoring/scheduling across logical chains), two methods exist to transfer Subscribe-type ctx info to Publish-type ctx:

1. Use `MergeSubscribeContextToPublishContext` method from `PublisherRef` or `ChannelHandleRef`:
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

2. Use `NewContextSharedPtr` method of `aimrt::channel::PublisherProxy` with Subscribe-type ctx parameter:
```cpp
aimrt::channel::PublisherProxy<BarMsg> publisher_proxy;

// Subscribe callback
void EventHandle(ContextRef subscribe_ctx, const std::shared_ptr<const FooMsg>& msg) {
    BarMsg new_msg;

    auto publishe_ctx = publisher_proxy.NewContextSharedPtr(subscribe_ctx);

    publisher_proxy.Publish(publishe_ctx, new_msg);
}
```


## Usage Example

Here is a simple example of publishing messages using the proxy-style interface:
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

Here is a simple example of subscribing to messages using the proxy-style interface:
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

