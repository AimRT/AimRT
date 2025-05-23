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

Protocols are used to define the message format for communication between endpoints. Generally, protocols are described using an IDL (Interface Description Language) that is independent of specific programming languages, and then converted into code for various languages using specific tools. Here we briefly introduce how AimRT officially supports the conversion of two IDLs into C++ code. For advanced usage, please refer to the official documentation of the respective IDL.

### Protobuf

[Protobuf](https://protobuf.dev/) is a lightweight, efficient data interchange format developed by Google for serializing structured data, and is a widely used IDL.

When using it, developers first need to define a `.proto` file containing the message structure. For example, `example.proto`:

```protobuf
syntax = "proto3";

message ExampleMsg {
  string msg = 1;
  int32 num = 2;
}
```

Then use the protoc tool provided by Protobuf to generate C++ code. For example:
```shell
protoc --cpp_out=. example.proto
```

This will generate `example.pb.h` and `example.pb.cc` files, containing the C++ classes and methods generated based on the defined message types.

Note that this native code generation approach is only meant to demonstrate the underlying principles to developers. In actual use, manual handling of dependencies and CMake encapsulation can be cumbersome. AimRT provides some encapsulation for this process, allowing developers to directly use the two CMake methods provided in the {{ '[ProtobufGenCode.cmake]({}/cmake/ProtobufGenCode.cmake)'.format(code_site_root_path_url) }} file:
- `add_protobuf_gencode_target_for_proto_path`: Generates C++ code for `.proto` files in a specified path, with the following parameters:
  - **TARGET_NAME**: The name of the generated CMake Target;
  - **PROTO_PATH**: The directory containing the protocol files;
  - **GENCODE_PATH**: The path to store the generated stub code;
  - **DEP_PROTO_TARGETS**: Dependent Proto CMake Targets;
  - **OPTIONS**: Additional parameters passed to protoc;
- `add_protobuf_gencode_target_for_one_proto_file`: Generates C++ code for a single `.proto` file, with the following parameters:
  - **TARGET_NAME**: The name of the generated CMake Target;
  - **PROTO_FILE**: The path to a single protocol file;
  - **GENCODE_PATH**: The path to store the generated stub code;
  - **DEP_PROTO_TARGETS**: Dependent Proto CMake Targets;
  - **OPTIONS**: Additional parameters passed to protoc;

Usage example:
```cmake
# Generate C++ code for all '.proto' files in the current folder
add_protobuf_gencode_target_for_proto_path(
  TARGET_NAME example_pb_gencode
  PROTO_PATH ${CMAKE_CURRENT_SOURCE_DIR}
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR})
```

After this, simply link the `example_pb_gencode` CMake Target to use the protocol. For example:
```cmake
target_link_libraries(my_lib PUBLIC example_pb_gencode)
```

### ROS2 Message

ROS2 Message is a structured data format used for communication and data exchange in ROS2. When using it, developers first need to define a ROS2 Package and then define a `.msg` file within it, such as `example.msg`:

```
int32   num
float32 num2
char    data
```

Then directly use the ROS2-provided CMake method `rosidl_generate_interfaces` to generate C++ code and CMake Targets for the message. For example:
```cmake
rosidl_generate_interfaces(
  example_msg_gencode
  "msg/example.msg"
)
```

After this, you can reference the relevant CMake Targets to use the generated C++ code. For details, please refer to the official ROS2 documentation and the examples provided by AimRT.## ChannelHandle

In AimRT, modules can obtain the `aimrt::channel::ChannelHandleRef` handle by calling the `GetChannelHandle()` interface of the `CoreRef` handle to utilize the Channel functionality. Its core interfaces are as follows:
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

Developers can call the `GetPublisher` and `GetSubscriber` methods in `ChannelHandleRef` to obtain `PublisherRef` and `SubscriberRef` type handles for specified Topic names, which are used for Channel publishing and subscribing, respectively. Notes on using these two methods:
  - These interfaces are thread-safe.
  - These interfaces can be used during both the `Initialize` and `Start` phases.

The `PublisherRef` and `SubscriberRef` handles provide a protocol-agnostic API interface. However, unless developers intend to use custom message types, they typically do not need to directly call these interfaces.

AimRT officially supports two protocol types: **Protobuf** and **Ros2 Message**, and provides Channel interface encapsulations for these two protocol types. Apart from the different protocol types, the overall API style of these two sets of Channel interfaces is consistent. Developers generally use these protocol-bound Channel interfaces directly. The corresponding CMake Targets must be referenced when using them:
- Protobuf Channel: Requires CMake reference to `aimrt::interface::aimrt_module_protobuf_interface`;
- Ros2 Channel: Requires CMake reference to `aimrt::interface::aimrt_module_ros2_interface`;

Developers can also use the `MergeSubscribeContextToPublishContext` method to pass context information from the subscribe side to the publish side, which can be used to connect the entire data pipeline. For details, refer to the Context section.

## Publish

AimRT provides two styles of interfaces for publishing a message: **Function Style** and **Proxy Style**:

- Function Style Interface:
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

- Proxy Class Style Interface:
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

The Proxy-style interface can bind type information and a default Context, offering more comprehensive functionality. However, the basic usage effects of both styles are consistent. Users need to follow two steps to achieve logical message publishing:
- **Step 1**: Use the `RegisterPublishType` method to register the message type:
  - Can only be registered during the `Initialize` phase;
  - Duplicate registration of the same type in a single `PublisherRef` is not allowed;
  - Returns false if registration fails;
- **Step 2**: Use the `Publish` method to publish data:
  - Data can only be published after the `Start` phase;
  - There are two `Publish` interfaces, one of which includes an additional Context parameter for passing extra information to the backend or downstream. For detailed Context descriptions, refer to subsequent sections;
  - When calling the `Publish` interface, developers must ensure that the passed Context and Msg remain unchanged until the `Publish` interface returns; otherwise, the behavior is undefined;

After a user `Publish`es a message, the specific Channel backend will handle the actual message publishing request. Depending on the backend implementation, this may block for some time, so the duration of the `Publish` method is undefined. However, generally, the Channel backend does not block the `Publish` method for too long. For more details, refer to the corresponding backend documentation.## Subscribe

Similar to the publish interface, AimRT provides two style types of interfaces for subscribing to a message: **function-style** and **Proxy-style**, along with two forms of callback functions: **smart pointer form** and **coroutine form**:

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

- Proxy-style interface:
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

The Proxy-type interface can bind type information and offers more comprehensive functionality. However, the basic usage effects of both style interfaces are consistent. When using the Subscribe interface, note the following:
- Subscription interfaces can only be called during the `Initialize` phase;
- Repeated subscriptions of the same type within a single `SubscriberRef` are not allowed;
- If subscription fails, it will return false;
- Two types of callback functions can be passed, one of which includes an additional Context parameter for passing extra information. For detailed explanations of Context, refer to subsequent sections;
- Lifecycle of Context and Msg:
  - For callbacks receiving Msg in smart pointer form, the lifecycle of Context and Msg will persist until the reference count of the Msg's smart pointer reaches zero and it is destructed;
  - For coroutine-form callbacks, the lifecycle of Context and Msg will persist until the coroutine exits;

Additionally, it's important to note which executor will execute the subscription callback. This depends on the specific Channel backend implementation and can only be determined during the runtime phase through configuration. Users should not make any assumptions when writing logic code. For detailed information, refer to the documentation of the corresponding backend.

The best practice is: if the task in the callback is very lightweight, such as simply setting a variable, it can be processed directly in the callback. However, if the task in the callback is heavier, it's better to schedule it to other dedicated task executors for processing.## Context

When publishing Channel messages, developers can pass an `aimrt::channel::Context`. When subscribing to Channel messages, they can also choose to receive an `aimrt::channel::ContextRef` in the callback. The `ContextRef` type is a reference to the `Context` type, and both share essentially the same interfaces. Their primary function is to carry Key-Val data for passing specific information downstream or to the Channel backend.

The interface is as follows:

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

When using Channel ctx of type `Context` or `ContextRef`, note the following:
- Channel ctx is divided into Publish-side and Subscribe-side types, determined during construction and unmodifiable, used for Publish and Subscribe scenarios respectively;
- Use the `SetMetaValue` and `GetMetaValue` methods to set and retrieve Key-Val values in the ctx, and `GetMetaKeys` to get all current Key values;

AimRT defines some special Keys in the {{ '[channel_context_base.h]({}/src/interface/aimrt_module_c_interface/channel/channel_context_base.h)'.format(code_site_root_path_url) }} file. When using these special Keys, certain rules should be followed. These special Keys include:
- **AIMRT_CHANNEL_CONTEXT_KEY_SERIALIZATION_TYPE**: Used to set the message serialization type, which must be a type supported by the registered type support;
- **AIMRT_CHANNEL_CONTEXT_KEY_BACKEND**: Used to pass the actual backend name to the Subscribe side;
- **AIMRT_CHANNEL_CONTEXT_KEY_TO_ADDR**: Used to pass the actual server address to the Publish side, formatted as: `backend://ip:port;backend://ip:port;...`, where `backend` is the backend type, and `ip` and `port` are the actual server addresses. Currently, three backend types are supported: `http`, `tcp`, and `udp`. If an address for a specific backend is included, the address specified in the configuration file will not be used. For example, if specified as `http://127.0.0.1:50090;tcp://127.0.0.1:50060`, the http and tcp addresses in the configuration file will be ignored, and messages will only be sent to these two addresses.

On the Publish side, `Context` is mainly used to pass special information to the AimRT framework and Channel backend when calling the `Publish` method. Note the following when using it:
- Developers can directly construct a `Context` type instance and are responsible for its lifecycle;
- Only Publish-type ctx can be passed to the `Publish` method;
- Each `Context` can only be used for one Publish process. After being passed to the `Publish` method, its state will be set to `Used`. If used for another Publish without `Reset`, the message will not be published correctly;
- The `Publish` method actually accepts `ContextRef` type as a parameter, but the `Context` type can be implicitly converted to `ContextRef`;
- Developers can set information in the ctx to pass to specific Channel backends. Different backends handle ctx information differently—some read specific Key-Val values to specialize transmission behavior, while others transparently pass all Key-Val information downstream. Refer to specific Channel backend documentation for details.

On the Subscribe side, developers can choose to receive a `ContextRef` type parameter in the callback handler. Note the following when using it:
- The lifecycle of the ctx passed to the callback handler is managed by the AimRT framework and is consistent with the Msg lifecycle;
- The ctx passed to the callback handler is of Subscribe type and in the `Used` state;
- The ctx passed to the callback handler may contain Key-Val information, which depends on the Channel backend. Refer to specific Channel backend documentation for details.

Additionally, in a complex business system, some subscribers may publish new messages downstream after receiving messages, forming many logical long chains. To connect these logical chains at the framework level for monitoring or scheduling purposes, specific information from Subscribe-type ctx needs to be synchronized to Publish-type ctx. There are two ways to achieve this:

1. Use the `MergeSubscribeContextToPublishContext` method provided by `PublisherRef` or `ChannelHandleRef`, for example:
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

2. Use the `NewContextSharedPtr` method of `aimrt::channel::PublisherProxy`, passing the Subscribe-type ctx as a parameter, for example:
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

Here is a simple example of publishing a message using the proxy-style interface:
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

Here is a simple example of subscribing to messages:
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