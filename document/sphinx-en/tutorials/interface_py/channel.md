# Channel

## Related Links

Reference examples:
- {{ '[examples_py_pb_chn_publisher_app.py]({}/src/examples/py/pb_chn/examples_py_pb_chn_publisher_app.py)'.format(code_site_root_path_url) }}
- {{ '[examples_py_pb_chn_subscriber_app.py]({}/src/examples/py/pb/chn/examples_py_pb_chn_subscriber_app.py)'.format(code_site_root_path_url) }}

## Protocol

Protocols are used to determine the message format for communication endpoints. Generally, protocols are described using an IDL (Interface Description Language) that is programming language-agnostic, then converted into code for various languages using specific tools.

### Protobuf

[Protobuf](https://protobuf.dev/) is a lightweight, efficient data interchange format developed by Google for serializing structured data, and is a widely used IDL.

The current version of AimRT Python only supports the protobuf protocol. Before using AimRT Python to publish/subscribe messages, users need to generate Python stub code based on the protobuf protocol.

During usage, developers first need to define a `.proto` file containing the message structure. For example, `example.proto`:

```protobuf
syntax = "proto3";

message ExampleMsg {
  string msg = 1;
  int32 num = 2;
}
```

Then use the official Protobuf protoc tool to convert it into Python code, for example:
```shell
protoc --python_out=. example.proto
```

This will generate an `example_pb2.py` file containing Python interfaces for the defined message types, which needs to be imported in our business code.

### ROS2 Message

ROS2 Message is a structured data format used for communication and data exchange in ROS2. During usage, developers first need to define a ROS2 Package containing a `.msg` file, such as `example.msg`:

```
int32   num
float32 num2
char    data
```

Then directly use ROS2's CMake method `rosidl_generate_interfaces` to generate C++ code and CMake Target for the message, for example:
```cmake
rosidl_generate_interfaces(
  example_ros2_msg_gencode
  "msg/example.msg"
)
```

After building, corresponding environment variables need to be set to use the generated message types in Python. Execute the following in aimrt's build directory:

```bash
source install/share/example_ros2/setup.bash
```

For more details about generating custom ROS2 Messages, please refer to the [ROS2 Official Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).

## ChannelHandle

Modules can obtain a `ChannelHandleRef` handle by calling the `GetChannelHandle()` interface of the `CoreRef` handle to use Channel functionality. Its core interfaces include:
- `GetPublisher(str)->PublisherRef`
- `GetSubscriber(str)->SubscriberRef`

Developers can call the `GetPublisher` and `GetSubscriber` methods in `ChannelHandleRef` to obtain `PublisherRef` and `SubscriberRef` type handles for specified Topic names, used for Channel publishing and subscribing respectively. Notes for using these methods:
  - These interfaces are thread-safe.
  - These interfaces can be used during both the `Initialize` and `Start` phases.

## Publish

If users need to publish a Msg, the main interfaces involved are:
- `aimrt_py.RegisterPublishType(publisher, msg_type)->bool`: Used to register this message type;
  - The first parameter `publisher` is a `PublisherRef` handle representing a Topic;
  - The second parameter `msg_type` is a `Protobuf` type;
  - The return value is a bool indicating whether registration was successful;
- `aimrt_py.Publish(publisher, msg, ctx | serialization_type)`: Used to publish messages;
  - The first parameter `publisher` is a `PublisherRef` handle representing a Topic;
  - The second parameter `msg` is a `Protobuf` type instance that must match the registered message type;
  - The third parameter can be a `Context` type instance, `ContextRef` handle, or `serialization_type` string to specify the message context or serialization type. `serialization_type` can only be `pb` or `json`. Both `ctx` and `serialization_type` can be empty, defaulting to pb serialization when empty;
  - This function also has the following overload:
    - `aimrt_py.Publish(publisher, ctx | serialization_type, msg)`

Users need two steps to implement logical message publishing:
- **Step 1**: Use the `aimrt_py.RegisterPublishType` method to register the protocol type;
  - Can only be registered during the `Initialize` phase;
  - Duplicate registration of the same type in a `PublisherRef` is not allowed;
  - Returns false if registration fails;
- **Step 2**: Use the `aimrt_py.Publish` method to publish data;
  - Can only publish data after the `Start` phase;
  - When calling the `Publish` interface, developers must ensure the Msg remains unchanged until the `Publish` interface returns, otherwise behavior is undefined;

After a user publishes a message, the specific Channel backend will handle the actual message publishing request. Depending on different backend implementations, this may block for some time, so the time consumed by the `Publish` method is undefined. However, generally Channel backends won't block the `Publish` method for too long. For details, please refer to the corresponding backend documentation.

## Subscribe

If users need to subscribe to a Msg, they need to use the following interface:
- `aimrt_py.Subscribe(subscriber, msg_type, handle)->bool`: Used to subscribe to a message type;
  - The first parameter `subscriber` is a `SubscriberRef` handle representing a Topic;
  - The second parameter `msg_type` is a `Protobuf` type;
  - The third parameter `handle` is a callback with signature `(msg)->void` or `(ctx_ref, msg)->void` for message processing. `msg` type is the subscribed `msg_type`, `ctx_ref` is the message context handle;
  - The return value is a bool indicating whether subscription was successful;

Notes:
- Subscription interfaces can only be called during `Initialize`;
- Duplicate subscription of the same type in a `SubscriberRef` is not allowed;
- Returns false if subscription fails;

Additionally, note that which executor will execute the subscription callback depends on the specific Channel backend implementation and can only be determined during runtime through configuration. Users should not make any assumptions when writing logic code. For details, please refer to the corresponding backend documentation.

Best practice is: If the task in the callback is very lightweight (e.g., just setting a variable), it can be processed directly in the callback; but if the callback task is heavy, it's better to schedule it to other dedicated task executors for processing.## Context

`Context` is a data structure in AimRT used for passing contextual information, which supports the following interfaces:
- `Reset()->void`: Resets the context, allowing it to be reused after resetting;
- `CheckUsed()->bool`: Checks whether the context has been used;
- `SetUsed()->void`: Marks the context as used;
- `GetType()->aimrt_channel_context_type_t`: Gets the context type;
- `SetMetaValue(key: str, value: str)->void`: Sets metadata;
- `GetMetaValue(key: str)->str`: Retrieves metadata;
- `GetMetaKeys()->List[str]`: Gets the list of all keys in the metadata key-value pairs;
- `SetSerializationType(serialization_type: str)->void`: Sets the serialization type;
- `GetSerializationType()->str`: Gets the serialization type;
- `ToString()->str`: Retrieves context information, returning highly readable information in string form;

`ContextRef` is a reference type of `Context`. Except for lacking the `Reset` interface, all other interfaces are identical to `Context`.

`aimrt_channel_context_type_t` is an enumeration type that defines the context type, with specific values being `AIMRT_CHANNEL_PUBLISHER_CONTEXT` or `AIMRT_CHANNEL_SUBSCRIBER_CONTEXT`, indicating whether it is a publisher or subscriber context.## Usage Example

Here is an example of using AimRT Python for Publish, obtaining the `CoreRef` handle through the Create Module approach. If the `CoreRef` handle is obtained in the `Initialize` method based on the `Module` mode, the usage is similar:
```python
import aimrt_py
import threading

from google.protobuf.json_format import MessageToJson
import event_pb2

def main():
    aimrt_core = aimrt_py.Core()

    # Initialize
    core_options = aimrt_py.CoreOptions()
    core_options.cfg_file_path = "path/to/cfg/xxx_cfg.yaml"
    aimrt_core.Initialize(core_options)

    # Create Module
    module_handle = aimrt_core.CreateModule("NormalPublisherPyModule")

    # Register publish type
    topic_name = "test_topic"
    publisher = module_handle.GetChannelHandle().GetPublisher(topic_name)
    assert publisher, f"Get publisher for topic '{topic_name}' failed."

    aimrt_py.RegisterPublishType(publisher, event_pb2.ExampleEventMsg)

    # Start
    thread = threading.Thread(target=aimrt_core.Start)
    thread.start()

    # Sleep for seconds
    time.sleep(1)

    # Publish event
    event_msg = event_pb2.ExampleEventMsg()
    event_msg.msg = "Publish without ctx or serialization_type"
    event_msg.num = 1
    aimrt_py.Publish(publisher, event_msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new pb event, data: {MessageToJson(event_msg)}")

    # Publish event with json serialization
    event_msg.msg = "Publish with json serialization"
    event_msg.num = 2
    aimrt_py.Publish(publisher, "json", event_msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new pb event, data: {MessageToJson(event_msg)}")

    # Publish event with context
    ctx = aimrt_py.Context()
    ctx.SetMetaValue("key1", "value1")
    event_msg.msg = "Publish with context"
    event_msg.num = 3
    aimrt_py.Publish(publisher, ctx, event_msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new pb event, data: {MessageToJson(event_msg)}")

    # Publish event with context ref
    ctx.Reset()  # Reset context, then it can be used again
    ctx_ref = aimrt_py.ContextRef(ctx)
    ctx_ref.SetMetaValue("key2", "value2")
    ctx_ref.SetSerializationType("json")
    event_msg.msg = "Publish with context ref"
    event_msg.num = 4
    aimrt_py.Publish(publisher, ctx_ref, event_msg)
    aimrt_py.info(module_handle.GetLogger(),
                  f"Publish new pb event, data: {MessageToJson(event_msg)}")

    # Sleep for seconds
    time.sleep(1)

    # Shutdown
    aimrt_core.Shutdown()

    thread.join()

if __name__ == '__main__':
    main()
```


Here is an example of using AimRT Python for Subscribe, obtaining the `CoreRef` handle through the Create Module approach. If the `CoreRef` handle is obtained in the `Initialize` method based on the `Module` mode, the usage is similar:

```python
import aimrt_py
import threading

from google.protobuf.json_format import MessageToJson
import event_pb2

global_aimrt_core = None


def signal_handler(sig, frame):
    global global_aimrt_core

    if (global_aimrt_core and (sig == signal.SIGINT or sig == signal.SIGTERM)):
        global_aimrt_core.Shutdown()
        return

    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    aimrt_core = aimrt_py.Core()

    global global_aimrt_core
    global_aimrt_core = aimrt_core

    # Initialize
    core_options = aimrt_py.CoreOptions()
    core_options.cfg_file_path = "path/to/cfg/xxx_cfg.yaml"
    aimrt_core.Initialize(core_options)

    # Create Module
    module_handle = aimrt_core.CreateModule("NormalSubscriberPyModule")

    # Subscribe
    topic_name = "test_topic"
    subscriber = module_handle.GetChannelHandle().GetSubscriber(topic_name)
    assert subscriber, f"Get subscriber for topic '{topic_name}' failed."

    def EventHandle(ctx_ref, msg):
        aimrt_py.info(module_handle.GetLogger(),
                      f"Get new pb event, ctx: {ctx_ref.ToString()}, data: {MessageToJson(msg)}")

    aimrt_py.Subscribe(subscriber, event_pb2.ExampleEventMsg, EventHandle)

    # Start
    thread = threading.Thread(target=aimrt_core.Start)
    thread.start()

    while thread.is_alive():
        thread.join(1.0)


if __name__ == '__main__':
    main()
```