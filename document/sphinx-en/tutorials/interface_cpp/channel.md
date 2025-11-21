# Channel

## Related Links

Code files:
- {{ '[aimrt_module_cpp_interface/channel/channel_context.h]({}/src/interface/aimrt_module_cpp_interface/channel/channel_context.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_cpp_interface/channel/channel_handle.h]({}/src/interface/aimrt_module_cpp_interface/channel/channel_handle.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_protobuf_interface/channel/protobuf_channel.h]({}/src/interface/aimrt_module_protobuf_interface/channel/protobuf_channel.h)'.format(code_site_root_path_url) }}
- {{ '[aimrt_module_ros2_interface/channel/ros2_channel.h]({}/src/interface/aimrt_module_ros2_interface/channel/ros2_channel.h)'.format(code_site_root_path_url) }}

Reference examples:
- {{ '[pb_chn]({}/src/examples/cpp/pb_chn)'.format(code_site_root_path_url) }}
  - {{ '[normal_publisher_module.cc]({}/src/examples/cpp/pb_chn/module/normal_publisher_module/normal_publisher_module.cc)'.format(code_site_root_path_url) }}
  - {{ '[normal_subscriber_module.cc]({}/src/examples/cpp/pb_chn/module/normal_subscriber_module/normal_subscriber_module.cc)'.format(code_site_root_path_url) }}
- {{ '[ros2_chn]({}/src/examples/cpp/ros2_chn)'.format(code_site_root_path_url) }}
  - {{ '[normal_publisher_module.cc]({}/src/examples/cpp/ros2_chn/module/normal_publisher_module/normal_publisher_module.cc)'.format(code_site_root_path_url) }}
  - {{ '[normal_subscriber_module.cc]({}/src/examples/cpp/ros2_chn/module/normal_subscriber_module/normal_subscriber_module.cc)'.format(code_site_root_path_url) }}

## Protocol

Protocols are used to determine the message format at each communication endpoint. Generally, protocols are described using an IDL (Interface Description Language) that is independent of any specific programming language, and then converted into code for each language by some tool. Here we briefly introduce how the two IDLs officially supported by AimRT are converted into C++ code. For advanced usage, please refer to the official documentation of the corresponding IDL.

### Protobuf

[Protobuf](https://protobuf.dev/) is a lightweight and efficient data exchange format for serializing structured data, developed by Google, and is a widely used IDL.

When using it, developers first need to define a `.proto` file in which a message structure is defined. For example, `example.proto`:


```protobuf
syntax = "proto3";

message ExampleMsg {
  string msg = 1;
  int32 num = 2;
}
```


Then use the protoc tool provided by the official Protobuf to generate C++ code, for example:

```shell
protoc --cpp_out=. example.proto
```


This will generate `example.pb.h` and `example.pb.cc` files, containing the C++ classes and methods generated according to the defined message types.

Please note that the above native code generation method is only to show developers the underlying principle. In actual use, you still need to manually handle dependencies and CMake packaging, which is cumbersome. AimRT has encapsulated this process to some extent. Developers can directly use the two CMake methods provided in {{ '[ProtobufGenCode.cmake]({}/cmake/ProtobufGenCode.cmake)'.format(code_site_root_path_url) }}:
- `add_protobuf_gencode_target_for_proto_path`: Generates C++ code for `.proto` files in a certain path, with the following parameters:
  - **TARGET_NAME**: The name of the generated CMake Target;
  - **PROTO_PATH**: The directory where the protocols are stored;
  - **GENCODE_PATH**: The path where the generated stub code will be stored;
  - **DEP_PROTO_TARGETS**: The dependent Proto CMake Targets;
  - **OPTIONS**: Other parameters passed to protoc;
- `add_protobuf_gencode_target_for_one_proto_file`: Generates C++ code for a single `.proto` file, with the following parameters:
  - **TARGET_NAME**: The name of the generated CMake Target;
  - **PROTO_FILE**: The path to a single protocol file;
  - **GENCODE_PATH**: The path where the generated stub code will be stored;
  - **DEP_PROTO_TARGETS**: The dependent Proto CMake Targets;
  - **OPTIONS**: Other parameters passed to protoc;

Usage example:

```cmake
# Generate C++ code for all '.proto' files in the current folder
add_protobuf_gencode_target_for_proto_path(
  TARGET_NAME example_pb_gencode
  PROTO_PATH ${CMAKE_CURRENT_SOURCE_DIR}
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR})
```


After that, just link the `example_pb_gencode` CMake Target to use the protocol. For example:

```cmake
target_link_libraries(my_lib PUBLIC example_pb_gencode)
```


### ROS2 Message

ROS2 Message is a structured data format used for communication and data exchange in ROS2. When using it, developers first need to define a ROS2 Package and define a `.msg` file within it, such as `example.msg`:


```
int32   num
float32 num2
char    data
```


Then directly use the CMake method `rosidl_generate_interfaces` provided by ROS2 to generate C++ code and CMake Target for the message, for example:

```cmake
rosidl_generate_interfaces(
  example_msg_gencode
  "msg/example.msg"
)
```


After that, you can reference the relevant CMake Target to use the generated C++ code. For details, please refer to the official ROS2 documentation and the Example provided by AimRT.## ChannelHandle

In AimRT, modules can obtain the `aimrt::channel::ChannelHandleRef` handle by calling the `GetChannelHandle()` interface of the `CoreRef` handle to use the Channel functionality. Its core interfaces are as follows:

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


Developers can call the `GetPublisher` and `GetSubscriber` methods in `ChannelHandleRef` to obtain `PublisherRef` and `SubscriberRef` type handles for a specified Topic name, used for Channel publishing and subscribing respectively. Notes on these two methods:
  - These two interfaces are thread-safe.
  - These two interfaces can be used in both the `Initialize` phase and the `Start` phase.

The `PublisherRef` and `SubscriberRef` handles provide a protocol-type-agnostic API interface, but developers only need to directly call the interfaces they provide if they want to use custom message types.

AimRT officially supports two protocol types: **Protobuf** and **Ros2 Message**, and provides Channel interface encapsulation for these two protocol types. Apart from the different protocol types, the overall API style of these two Channel interfaces is consistent, and developers generally use these protocol-type-bound Channel interfaces directly. When using them, the corresponding CMake Targets need to be referenced:
- Protobuf Channel: requires CMake reference to `aimrt::interface::aimrt_module_protobuf_interface`;
- Ros2 Channel: requires CMake reference to `aimrt::interface::aimrt_module_ros2_interface`;

Developers can also use the `MergeSubscribeContextToPublishContext` method to pass context information from the subscribe side to the publish side, which can be used to link the entire data chain. For details, please refer to the Context chapter.

## Publish

AimRT provides **function-style** and **Proxy-style** interfaces to publish a message:

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


- Proxy class-style interface:

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


The Proxy-type interface can bind type information and a default Context, offering more complete functionality. However, the basic usage effect of both style interfaces is consistent. Users need two steps to achieve logical-level message publishing:
- **Step 1**: Register the message type using the `RegisterPublishType` method:
  - Can only be registered during the `Initialize` phase;
  - Not allowed to repeatedly register the same type in a `PublisherRef`;
  - If registration fails, it will return false;
- **Step 2**: Publish data using the `Publish` method:
  - Data can only be published after the `Start` phase;
  - There are two `Publish` interfaces, one of which has an additional Context parameter for passing some extra information to the backend or downstream. Detailed explanation of Context is in subsequent chapters;
  - When calling the `Publish` interface, developers should ensure that the passed Context and Msg do not change before the `Publish` interface returns, otherwise the behavior is undefined;

After a user `Publish`es a message, the specific Channel backend will handle the actual message publishing request. Depending on the implementation of different backends, it may block for a period of time, so the time consumed by the `Publish` method is undefined. However, generally speaking, Channel backends will not block the `Publish` method for too long. For detailed information, please refer to the documentation of the corresponding backend.## Subscribe

Like the publishing interface, AimRT provides both **function-style** and **Proxy-style** interfaces for subscribing to a message, along with two callback forms: **smart pointer form** and **coroutine form**:

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


The Proxy-style interface can bind type information and is more feature-rich. However, the basic usage effects of both styles are the same. When using the Subscribe interface, note the following:
- Subscription interfaces can only be called during the `Initialize` phase;
- Repeated subscriptions to the same type within a single `SubscriberRef` are not allowed;
- If subscription fails, it will return false;
- Two types of callback functions can be passed, one of which has an additional Context parameter for passing some extra information. See subsequent chapters for detailed explanation of Context;
- Lifecycle of Context and Msg:
  - For callbacks receiving Msg in smart pointer form, the lifecycle of Context and Msg will last until the smart pointer reference count of Msg reaches zero and it is destructed;
  - For coroutine-style callbacks, the lifecycle of Context and Msg will last until the coroutine exits;

Additionally, it should be noted that which executor will run the subscription callback depends on the specific Channel backend implementation and can only be determined through configuration at runtime. Users should not make any assumptions when writing logic code. Please refer to the corresponding backend documentation for detailed information.

Best practice: If the task in the callback is very lightweight, such as just setting a variable, it can be handled directly in the callback; but if the task in the callback is relatively heavy, it's better to schedule it to another executor specifically designed for task execution.## Context

When developers publish a Channel message, they can pass an `aimrt::channel::Context`; when subscribing to a Channel message, they can also choose to receive an `aimrt::channel::ContextRef` in the callback. The `ContextRef` type is a reference to the `Context` type, and the interfaces they provide are essentially the same. Their main function is to carry some key-value data to pass specific information downstream or to the Channel backend.

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


When using the Channel ctx of type `Context` or `ContextRef`, note the following:
- Channel ctx is divided into Publish-side and Subscribe-side types, determined at construction and cannot be changed; they are used for Publish and Subscribe scenarios respectively;
- You can use the `SetMetaValue` and `GetMetaValue` methods to set and retrieve key-value pairs in the ctx, and use `GetMetaKeys` to get all current keys;

AimRT defines some special keys in the file {{ '[channel_context_base.h]({}/src/interface/aimrt_module_c_interface/channel/channel_context_base.h)'.format(code_site_root_path_url) }}. When using these special keys in business code, certain rules must be followed. These special keys include:
- **AIMRT_CHANNEL_CONTEXT_KEY_SERIALIZATION_TYPE**: Used to set the serialization type of the message; it must be a type supported in the registered type support;
- **AIMRT_CHANNEL_CONTEXT_KEY_BACKEND**: Used to pass the actual backend name handling the request to the Subscribe side;
- **AIMRT_CHANNEL_CONTEXT_KEY_TO_ADDR**: Used to pass the actual server address handling the request to the Publish side, in the format: `backend://ip:port;backend://ip:port;...`, where `backend` is the backend type, and `ip` and `port` are the actual server addresses. Currently, `http`, `tcp`, and `udp` backend types are supported. When an address for a backend is included, the address specified in the configuration file will no longer be used for sending. For example, if set to `http://127.0.0.1:50090;tcp://127.0.0.1:50060`, the http and tcp addresses specified in the configuration file will not be used, and messages will only be sent to these two addresses.
- **AIMRT_CHANNEL_CONTEXT_KEY_PUB_SEQ**: Used to transmit the publishing sequence number of the message in Publish/Subscribe scenarios. Within the same Topic, this value is automatically maintained by the framework as a monotonically increasing sequence (in single publisher scenarios, it can be considered unique). Business logic can use this field to analyze packet loss, out-of-order delivery, etc.
- **AIMRT_CHANNEL_CONTEXT_KEY_PUB_TIMESTAMP**: Used to transmit the publish timestamp of the message in Publish/Subscribe scenarios, in nanoseconds. Typically, the framework automatically fills this field with the current time at the moment of publishing (or relays the source timestamp from upstream middleware, such as the `source_timestamp` in ROS 2). Some plugins use this field when writing data to files (such as the `publishTime` field in MCAP). Business logic can use this field to calculate end-to-end latency or similar metrics.

On the Publish side, the `Context` is mainly used to pass some special information to the AimRT framework and Channel backend when calling the `Publish` method. Note the following:
- Developers can directly construct an instance of type `Context` and manage its lifecycle themselves;
- Only a Publish-side ctx can be passed to the `Publish` method;
- Each `Context` can only be used for one Publish process. After being passed to the `Publish` method, its state will be set to `Used`. If it is used for the next Publish without being `Reset`, the message will not be published correctly;
- The `Publish` method actually accepts a parameter of type `ContextRef`, but the `Context` type can be implicitly converted to the `ContextRef` type;
- Developers can set some information in the ctx to pass to the specific Channel backend. Different backends handle the information in the ctx differently; some may read specific key-value pairs to specialize transmission behavior, while others may transparently pass all key-value information downstream. For specific handling methods, please refer to the documentation of the particular Channel backend.

On the Subscribe side, developers can choose to receive a parameter of type `ContextRef` in the callback handler. Note the following:
- The lifecycle of the ctx passed to the callback handler is managed by the AimRT framework and is consistent with the lifecycle of the Msg;
- The ctx passed to the callback handler is of Subscribe type and is in the `Used` state;
- The ctx passed to the callback handler may contain some key-value information; which specific information is passed is determined by the Channel backend. Please refer to the documentation of the particular Channel backend.

Additionally, in a complex business system, some subscribers will publish new messages downstream after receiving a message, resulting in many logical long chains. To connect these logical chains at the framework level to implement monitoring and scheduling functions, it is necessary to synchronize specific information from the Subscribe-side ctx to the Publish-side ctx. There are two ways to do this:

1. You can use the `MergeSubscribeContextToPublishContext` method provided by the `PublisherRef` or `ChannelHandleRef` type, for example:

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


2. You can use the `NewContextSharedPtr` method of `aimrt::channel::PublisherProxy`, passing the Subscribe-side ctx as a parameter to the method, for example:

```cpp
aimrt::channel::PublisherProxy<BarMsg> publisher_proxy;

// Subscribe callback
void EventHandle(ContextRef subscribe_ctx, const std::shared_ptr<const FooMsg>& msg) {
    BarMsg new_msg;

    auto publishe_ctx = publisher_proxy.NewContextSharedPtr(subscribe_ctx);

    publisher_proxy.Publish(publishe_ctx, new_msg);
}
```
## Usage Examples

Here is a simple example of publishing a message, based on the proxy-style interface:

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
