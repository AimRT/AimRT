# Channel


## Related Links

Reference examples:
- {{ '[examples_py_pb_chn_publisher_app.py]({}/src/examples/py/pb_chn/examples_py_pb_chn_publisher_app.py)'.format(code_site_root_path_url) }}
- {{ '[examples_py_pb_chn_subscriber_app.py]({}/src/examples/py/pb_chn/examples_py_pb_chn_subscriber_app.py)'.format(code_site_root_path_url) }}

## Protocol

A protocol is used to determine the message format for all communicating parties. Generally, protocols are described using a language-agnostic IDL (Interface Description Language) and then converted into code for each language by some tool.

### Protobuf

[Protobuf](https://protobuf.dev/) is a lightweight, efficient data exchange format for serializing structured data, developed by Google and widely used as an IDL.

The current AimRT Python release only supports the protobuf protocol. Before using AimRT Python to send/subscribe to messages, users must first generate some Python stub code based on the protobuf protocol.

To use it, developers first define a `.proto` file in which a message structure is defined. For example, `example.proto`:


```protobuf
syntax = "proto3";

message ExampleMsg {
  string msg = 1;
  int32 num = 2;
}
```


Then use the official protoc tool provided by Protobuf to convert it and generate Python code, for example:

```shell
protoc --python_out=. example.proto
```


This will produce the `example_pb2.py` file, which contains Python interfaces generated from the defined message types. Our business code needs to import this file.

### ROS2 Message

ROS2 Message is a structured data format used for communication and data exchange in ROS2. To use it, developers first create a ROS2 Package and define a `.msg` file within it, such as `example.msg`:


```
int32   num
float32 num2
char    data
```


Then directly use the CMake method provided by ROS2, `rosidl_generate_interfaces`, to generate C++ code and CMake Targets for the message, for example:

```cmake
rosidl_generate_interfaces(
  example_ros2_msg_gencode
  "msg/example.msg"
)
```


After building, you also need to set the corresponding environment variables before the generated message types can be used in Python. In the aimrt build directory, execute:


```bash
source install/share/example_ros2/setup.bash
```


For more detailed information on generating custom ROS2 Messages, please refer to the [ROS2 official documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).

## ChannelHandle

A module can obtain a `ChannelHandleRef` handle by calling the `GetChannelHandle()` interface of the `CoreRef` handle to use Channel functionality. Its core interfaces are:
- `GetPublisher(str)->PublisherRef`
- `GetSubscriber(str)->SubscriberRef`

Developers can call the `GetPublisher` and `GetSubscriber` methods in `ChannelHandleRef` to obtain `PublisherRef` and `SubscriberRef` type handles for a specified Topic name, used for Channel publishing and subscribing respectively. Usage notes for these two methods are as follows:
  - These two interfaces are thread-safe.
  - These two interfaces can be used during the `Initialize` phase and the `Start` phase.

## Publish

If a user needs to publish a Msg, the main interfaces involved are the following two:
- `aimrt_py.RegisterPublishType(publisher, msg_type)->bool`: Used to register this message type;
  - The first parameter `publisher` is a `PublisherRef` handle representing a certain Topic;
  - The second parameter `msg_type` is a `Protobuf` type;
  - The return value is a bool indicating whether the registration was successful;
- `aimrt_py.Publish(publisher, msg, ctx | serialization_type)`: Used to publish a message;
  - The first parameter `publisher` is a `PublisherRef` handle representing a certain Topic;
  - The second parameter `msg` is an instance of a `Protobuf` type, which must correspond to the message type registered;
  - The third parameter can be a `Context` type instance, a `ContextRef` handle, or a `serialization_type` string to specify the message's context or serialization type. `serialization_type` can only be set to `pb` or `json`. Both `ctx` and `serialization_type` can be empty; when empty, pb serialization is used by default;
  - This function also has the following overload:
    - `aimrt_py.Publish(publisher, ctx | serialization_type, msg)`

Users need two steps to implement message publishing at the logical level:
- **Step 1**: Use the `aimrt_py.RegisterPublishType` method to register the protocol type;
  - Registration can only be done during the `Initialize` phase;
  - Re-registering the same type in a `PublisherRef` is not allowed;
  - If registration fails, it returns false;
- **Step 2**: Use the `aimrt_py.Publish` method to publish data;
  - Data can only be published after the `Start` phase;
  - When calling the `Publish` interface, the developer must ensure that the Msg passed in does not change until the `Publish` interface returns; otherwise, the behavior is undefined;

After a user `Publish`es a message, a specific Channel backend will handle the actual message publishing request. Depending on the backend implementation, this may block for a period of time, so the time consumed by the `Publish` method is undefined. However, generally, Channel backends do not block the `Publish` method for long. For detailed information, please refer to the documentation of the corresponding backend.

## Subscribe

If a user needs to subscribe to a Msg, the following interface must be used:
- `aimrt_py.Subscribe(subscriber, msg_type, handle)->bool`: Used to subscribe to a message type;
  - The first parameter `subscriber` is a `SubscriberRef` handle representing a certain Topic;
  - The second parameter `msg_type` is a `Protobuf` type;
  - The third parameter `handle` is a message processing callback with the signature `(msg)->void` or `(ctx_ref, msg)->void`. The type of `msg` is the `msg_type` type passed during subscription, and `ctx_ref` is the message's context handle;
  - The return value is a bool indicating whether the registration was successful;

- The subscription interface can only be called during `Initialize`;
- Re-subscribing to the same type in a `SubscriberRef` is not allowed;
- If subscription fails, it returns false;

Additionally, note that which executor runs the subscription callback depends on the specific Channel backend implementation and can only be determined at runtime through configuration. Users should not make any assumptions when writing logical code. For detailed information, please refer to the documentation of the corresponding backend.

Best practice: If the task in the callback is very lightweight, such as just setting a variable, it can be handled directly in the callback; but if the task in the callback is relatively heavy, it is best to schedule it to another executor dedicated to handling tasks.## Context

`Context` is a data structure in AimRT used to pass contextual information. The interfaces it supports are as follows:
- `Reset()->void` : Resets the context; after resetting, the context can be reused;
- `CheckUsed()->bool` : Checks whether the context has been used;
- `SetUsed()->void` : Sets the context as used;
- `GetType()->aimrt_channel_context_type_t` : Gets the context type;
- `SetMetaValue(key: str, value: str)->void` : Sets metadata;
- `GetMetaValue(key: str)->str` : Gets metadata;
- `GetMetaKeys()->List[str]` : Gets a list of keys from all metadata key-value pairs;
- `SetSerializationType(serialization_type: str)->void` : Sets the serialization type;
- `GetSerializationType()->str` : Gets the serialization type;
- `ToString()->str` : Gets context information, returning human-readable information in string form;

`ContextRef` is the reference type of `Context`. Except for not having the `Reset` interface, all other interfaces are identical to `Context`.

`aimrt_channel_context_type_t` is an enumeration type that defines the context type. The specific values are `AIMRT_CHANNEL_PUBLISHER_CONTEXT` or `AIMRT_CHANNEL_SUBSCRIBER_CONTEXT`, indicating whether this is a publisher or subscriber context.## Usage Examples

Below is an example of using AimRT Python to Publish, obtaining the `CoreRef` handle via the Create Module approach. If you are using the `Module` pattern and obtaining the `CoreRef` handle in the `Initialize` method, the usage is similar:

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


Below is an example of using AimRT Python to Subscribe, obtaining the `CoreRef` handle via the Create Module approach. If you are using the `Module` pattern and obtaining the `CoreRef` handle in the `Initialize` method, the usage is similar:

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
``````
    while thread.is_alive():
        thread.join(1.0)


if __name__ == '__main__':
    main()
```