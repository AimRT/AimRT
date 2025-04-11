

# Channel

## Related Links

Reference Examples:
- {{ '[examples_py_pb_chn_publisher_app.py]({}/src/examples/py/pb_chn/examples_py_pb_chn_publisher_app.py)'.format(code_site_root_path_url) }}
- {{ '[examples_py_pb_chn_subscriber_app.py]({}/src/examples/py/pb_chn/examples_py_pb_chn_subscriber_app.py)'.format(code_site_root_path_url) }}

## Protocol

Protocols are used to determine the message format for all communication endpoints. Generally, protocols are described using an IDL (Interface Description Language) that is programming language-agnostic, then converted into code for various languages using specific tools.

### Protobuf

[Protobuf](https://protobuf.dev/) is a lightweight, efficient data interchange format developed by Google for serializing structured data, and is a widely used IDL.

The current version of AimRT Python only supports the protobuf protocol. Before using AimRT Python to send/subscribe messages, users need to generate Python stub code based on the protobuf protocol.

During usage, developers first need to define a `.proto` file containing message structures. For example `example.proto`:

```proto3
```protobuf
syntax = "proto3";

message ExampleMsg {
  string msg = 1;
  int32 num = 2;
}
```
```

Then use the protoc tool provided by Protobuf to generate Python code. For example:
```bash
```shell
protoc --python_out=. example.proto
```
```

This generates an `example_pb2.py` file containing Python interfaces for the defined message types. Our business code needs to import this file.

### ROS2 Message

ROS2 Message is a structured data format used for communication and data exchange in ROS2. During usage, developers first need to define a ROS2 Package containing a `.msg` file, such as `example.msg`:

```text
```
int32   num
float32 num2
char    data
```
```

Then use ROS2's CMake method `rosidl_generate_interfaces` to generate C++ code and CMake targets for the message. For example:
```cmake
```cmake
rosidl_generate_interfaces(
  example_ros2_msg_gencode
  "msg/example.msg"
)
```
```

After building, environment variables need to be configured for using the generated message types in Python. Execute in the aimrt build directory:

```bash
```bash
source install/share/example_ros2/setup.bash
```
```

For more details about generating custom ROS2 Messages, refer to [ROS2 Official Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).

## ChannelHandle

Modules can obtain a `ChannelHandleRef` handle by calling the `GetChannelHandle()` interface of the `CoreRef` handle to use Channel functionality. Its core interfaces include:
- `GetPublisher(str)->PublisherRef`
- `GetSubscriber(str)->SubscriberRef`

Developers can call the `GetPublisher` and `GetSubscriber` methods in `ChannelHandleRef` to obtain `PublisherRef` and `SubscriberRef` type handles for specified Topic names, used for channel publishing and subscription respectively. Notes for using these methods:
  - These interfaces are thread-safe
  - These interfaces can be used during both `Initialize` and `Start` phases

## Publish

Key interfaces involved in message publishing:
- `aimrt_py.RegisterPublishType(publisher, msg_type)->bool`: Registers message type
  - First parameter `publisher`: A `PublisherRef` handle representing a Topic
  - Second parameter `msg_type`: A `Protobuf` type
  - Returns bool indicating registration success
- `aimrt_py.Publish(publisher, msg, ctx | serialization_type)`: Publishes message
  - First parameter `publisher`: A `PublisherRef` handle representing a Topic
  - Second parameter `msg`: A `Protobuf` instance matching registered type
  - Third parameter: Can be `Context` instance, `ContextRef` handle, or `serialization_type` string (only `pb` or `json`). Defaults to pb serialization when empty
  - Overloads:
    - `aimrt_py.Publish(publisher, ctx | serialization_type, msg)`

Implementation steps:
- **Step 1**: Register protocol type using `aimrt_py.RegisterPublishType`
  - Only allowed during `Initialize` phase
  - Duplicate registration on same `PublisherRef` prohibited
  - Returns false on failure
- **Step 2**: Publish data using `aimrt_py.Publish`
  - Only allowed after `Start` phase
  - Developers must ensure message remains unchanged until `Publish` returns

After publishing, the Channel backend handles message distribution. Publishing duration is undefined but typically brief. Refer to backend documentation for details.

## Subscribe

Subscription interface:
- `aimrt_py.Subscribe(subscriber, msg_type, handle)->bool`: Subscribes to message type
  - First parameter `subscriber`: A `SubscriberRef` handle representing a Topic
  - Second parameter `msg_type`: A `Protobuf` type
  - Third parameter `handle`: Callback with signature `(msg)->void` or `(ctx_ref, msg)->void`
  - Returns bool indicating subscription success

Notes:
- Subscription interface only available during `Initialize`
- Duplicate subscriptions on same `SubscriberRef` prohibited
- Returns false on failure

Callback execution depends on Channel backend implementation and runtime configuration. Best practices:
- Handle lightweight tasks directly in callback
- Schedule heavy tasks to dedicated executors

For detailed executor behavior, refer to backend documentation.

## Context

`Context` is a data structure in AimRT for passing contextual information, supporting the following interfaces:
- `Reset()->void`: Reset the context, allowing it to be reused after reset
- `CheckUsed()->bool`: Check if the context has been used
- `SetUsed()->void`: Mark the context as used
- `GetType()->aimrt_channel_context_type_t`: Get context type
- `SetMetaValue(key: str, value: str)->void`: Set metadata
- `GetMetaValue(key: str)->str`: Get metadata value
- `GetMetaKeys()->List[str]`: Get list of all metadata keys
- `SetSerializationType(serialization_type: str)->void`: Set serialization type
- `GetSerializationType()->str`: Get serialization type
- `ToString()->str`: Get human-readable context information as string

`ContextRef` is the reference type of `Context`, sharing all interfaces except `Reset` with `Context`.

`aimrt_channel_context_type_t` is an enumeration type defining context types with values:
- `AIMRT_CHANNEL_PUBLISHER_CONTEXT`
- `AIMRT_CHANNEL_SUBSCRIBER_CONTEXT`
Indicating whether it's a publisher or subscriber context.

## Usage Examples

The following is an example of using AimRT Python for Publish, obtaining the `CoreRef` handle via the Create Module approach. If obtaining the `CoreRef` handle in the `Initialize` method based on `Module` mode, the usage is similar:
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

The following is an example of using AimRT Python for Subscribe, obtaining the `CoreRef` handle via the Create Module approach. If obtaining the `CoreRef` handle in the `Initialize` method based on `Module` mode, the usage is similar:

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

