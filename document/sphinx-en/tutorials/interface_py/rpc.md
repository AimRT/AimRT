

# Rpc

## Related Links

Reference Examples:
- {{ '[examples_py_pb_rpc_client_app.py]({}/src/examples/py/pb_rpc/examples_py_pb_rpc_client_app.py)'.format(code_site_root_path_url) }}
- {{ '[examples_py_pb_rpc_server_app.py]({}/src/examples/py/pb_rpc/examples_py_pb_rpc_server_app.py)'.format(code_site_root_path_url) }}
- {{ '[examples_py_ros2_rpc_client_app.py]({}/src/examples/py/ros2_rpc/examples_py_ros2_rpc_client_app.py)'.format(code_site_root_path_url) }}
- {{ '[examples_py_ros2_rpc_server_app.py]({}/src/examples/py/ros2_rpc/examples_py_ros2_rpc_server_app.py)'.format(code_site_root_path_url) }}

## Protocol

Protocols are used to determine the message format between clients and servers in RPC. Generally, protocols are described using an IDL (Interface Description Language) that is independent of specific programming languages, then converted into code for various languages using specific tools. For RPC, this requires two steps:
- As introduced in the [Channel](./channel.md) chapter, developers first need to use official tools to generate code for **message types** in the protocol file for target programming languages;
- Developers need to use tools provided by AimRT to generate code for **service definitions** in the protocol file for target programming languages;

### Protobuf

[Protobuf](https://protobuf.dev/) is a lightweight, efficient data interchange format developed by Google for serializing structured data, widely used as an IDL. It can describe message structures and also uses the `service` statement to define RPC services.

When using it, developers first define a `.proto` file containing message structures and RPC services. For example `rpc.proto`:

```protobuf
syntax = "proto3";

package example;

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

Then use the official protoc tool from Protobuf to generate Python code for message structures:
```shell
protoc --python_out=. rpc.proto
```

This generates the `rpc_pb2.py` file containing Python interfaces for the defined message types.

Next, use the protoc plugin provided by AimRT to generate Python stub code for service definitions:
```shell
protoc --aimrt_rpc_out=. --plugin=protoc-gen-aimrt_rpc=./protoc_plugin_py_gen_aimrt_py_rpc.py rpc.proto
```

This generates the `rpc_aimrt_rpc_pb2.py` file containing Python interfaces for the defined services. Our business code needs to import this file.

### ROS2 Srv

ROS2 Srv is a format used for RPC definition in ROS2. When using it, developers need to first define a ROS2 Package containing a `.srv` file, such as `example.srv`:

```
byte[]  data
---
int64   code
```

Where `---` separates Req and Rsp definitions.

The aimrt_py installation automatically provides a command-line tool `aimrt_py-gen-ros2-rpc` to generate AimRT Python RPC stub code from ROS2 Srv files.

```shell
aimrt_py-gen-ros2-rpc --pkg_name=example_pkg --srv_file=./example.srv --output_path=./
```

Where pkg_name is the ROS2 Package name, srv_file is the path to the ROS2 Srv file, and output_path is the output directory. This generates an `example_aimrt_rpc_ros2.py` file containing Python interfaces for the defined services. Our business code needs to import this file.

Note that `aimrt_py-gen-ros2-rpc` only generates Req and Rsp definitions, not message structure code. Message structure code still needs to be generated separately by building the ROS2 Package (refer to [ROS2 Official Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)).

## RpcHandle

Modules can obtain an `RpcHandleRef` handle by calling the `GetRpcHandle()` interface of the `CoreRef` handle. Generally, developers don't use `RpcHandleRef` interfaces directly, but instead use generated stub code that wraps `RpcHandleRef`, then use these wrapped interfaces in business code.

The specific forms of these wrapped interfaces will be introduced in later chapters. Developers using RPC functionality should follow these steps:
- Client side:
  - During `Initialize` phase: Call **RPC Client method registration** interfaces;
  - During `Start` phase: Call **RPC Invoke** interfaces to implement RPC calls;
- Server side:
  - During `Initialize` phase: Call **RPC Server service registration** interfaces;

## RpcStatus

During RPC calls or processing, users can obtain error information through an `RpcStatus` variable, which provides these interfaces:
- `OK()->bool`: Whether successful;
- `Code()->int`: Error code;
- `ToString()->str`: Convert to string;

The `RpcStatus` type is very lightweight, containing only an error code field. Users can set this code via constructor or Set methods, and retrieve it via Get methods. Error code enumerations can be found in {{ '[rpc_status_base.h]({}/src/interface/aimrt_module_c_interface/rpc/rpc_status_base.h)'.format(code_site_root_path_url) }}.

Note that `RpcStatus` errors generally indicate framework-level issues like service not found, network errors, or serialization errors, helping developers troubleshoot framework problems. For business-level errors, developers should add corresponding fields in business packages.

## RpcContext

RpcContext is the context information during RPC calls. Developers can set contextual information such as timeout duration and metadata during RPC calls. Specific interfaces include:
- `CheckUsed()->bool`: Check if the Context has been used
- `SetUsed()->None`: Mark the Context as used
- `Reset()->None`: Reset the Context
- `GetType()->aimrt_rpc_context_type_t`: Get context type
- `Timeout()->datetime.timedelta`: Get timeout duration
- `SetTimeout(timeout: datetime.timedelta)->None`: Set timeout duration
- `SetMetaValue(key: str, value: str)->None`: Set metadata
- `GetMetaValue(key: str)->str`: Get metadata value
- `GetMetaKeys()->List[str]`: Get list of all metadata keys
- `SetToAddr(addr: str)->None`: Set target address
- `GetToAddr()->str`: Get target address
- `SetSerializationType(serialization_type: str)->None`: Set serialization type
- `GetSerializationType()->str`: Get serialization type
- `GetFunctionName()->str`: Get function name
- `SetFunctionName(func_name: str)->None`: Set function name
- `ToString()->str`: Get human-readable context information as string

`RpcContextRef` is the reference type of `RpcContext`, sharing all interfaces except `Reset` with `RpcContext`.

`aimrt_rpc_context_type_t` is an enumeration type defining context types, with possible values `AIMRT_RPC_CLIENT_CONTEXT` or `AIMRT_RPC_SERVER_CONTEXT`, indicating whether it's client-side or server-side context.

## Client

In AimRT Python RPC stub code generated by the tool (e.g., `xxx_aimrt_rpc_pb2.py` files), the `XXXProxy` type is provided for developers to initiate RPC calls. This is a synchronous interface that blocks the current thread until receiving a response or timing out.

Using the Proxy for RPC calls typically involves these steps:
- **Step 0**: Reference the stub code package generated from the protobuf protocol (e.g., `xxx_aimrt_rpc_pb2.py`)
- **Step 1**: Call the Proxy's `RegisterClientFunc` static method during `Initialize` phase to register RPC Client
- **Step 2**: Initiate RPC call in business logic during `Start` phase:
  - **Step 2-1**: Create a Proxy instance with `RpcHandleRef` as constructor parameter
  - **Step 2-2**: Create Request object and populate its content
  - **Step 2-3**: [Optional] Create context and set timeout/metadata
  - **Step 2-4**: Invoke RPC call through proxy with context and Request, maintaining validity of context and Request throughout the call. Wait synchronously for completion and obtain status/Response
  - **Step 2-5**: Parse status and Response

Below is an example of using AimRT Python for RPC Client calls based on protobuf protocol, obtaining `CoreRef` handle via Create Module method. The usage is similar when obtaining `CoreRef` handle in `Initialize` method under `Module` mode:

```python
import aimrt_py
import threading
import time
import datetime

from google.protobuf.json_format import MessageToJson
import rpc_pb2
import rpc_aimrt_rpc_pb2

def main():
    aimrt_core = aimrt_py.Core()

    # Initialize
    core_options = aimrt_py.CoreOptions()
    core_options.cfg_file_path = "path/to/cfg/xxx_cfg.yaml"
    aimrt_core.Initialize(core_options)

    # Create Module
    module_handle = aimrt_core.CreateModule("NormalRpcClientPyModule")

    # Register rpc client
    rpc_handle = module_handle.GetRpcHandle()
    ret = rpc_aimrt_rpc_pb2.ExampleServiceProxy.RegisterClientFunc(rpc_handle)
    assert ret, "Register client failed."

    # Start
    thread = threading.Thread(target=aimrt_core.Start)
    thread.start()

    # Sleep for seconds
    time.sleep(1)

    # Call rpc
    proxy = rpc_aimrt_rpc_pb2.ExampleServiceProxy(rpc_handle)

    req = rpc_pb2.GetFooDataReq()
    req.msg = "example msg"

    ctx = aimrt_py.RpcContext()
    ctx.SetTimeout(datetime.timedelta(seconds=30))
    ctx.SetMetaValue("key1", "value1")
    status, rsp = proxy.GetFooData(ctx, req)

    aimrt_py.info(module_handle.GetLogger(),
                  f"Call rpc done, "
                  f"status: {status.ToString()}, "
                  f"req: {MessageToJson(req)}, "
                  f"rsp: {MessageToJson(rsp)}")

    # Shutdown
    aimrt_core.Shutdown()

    thread.join()

if __name__ == '__main__':
    main()
```

## Server

In the code generated by the AimRT Python RPC stub tool, such as the `xxx_aimrt_rpc_pb2.py` file, a Service base class inheriting from `aimrt_py.ServiceBase` is provided. Developers need to inherit this Service base class and implement its virtual interfaces. These Service interfaces are synchronous - developers must complete all operations blocking in the handle method and finally return the response packet.

Using this interface to provide RPC services generally involves the following steps:
- **Step 0**: Import the stub code package generated from the protobuf protocol, e.g., `xxx_aimrt_rpc_pb2.py`;
- **Step 1**: Developers implement an Impl class inheriting from `XXXService` in the package, and implement its virtual interfaces in the form of `(ctx, req)->status, rsp`;
  - **Step 1-1**: Parse Ctx and Req, then populate Rsp;
  - **Step 1-2**: Return `RpcStatus` and Rsp;
- **Step 2**: Call the `RegisterService` method of `RpcHandleRef` during the `Initialize` phase to register the RPC Service;

Below is an example of using AimRT Python for RPC Service processing based on protobuf protocol, obtaining the `CoreRef` handle through Create Module mode. The usage is similar when obtaining the `CoreRef` handle in the `Initialize` method based on `Module` mode:

```python
import aimrt_py
import threading
import signal

from google.protobuf.json_format import MessageToJson
import rpc_pb2
import rpc_aimrt_rpc_pb2


global_aimrt_core = None


def signal_handler(sig, frame):
    global global_aimrt_core

    if (global_aimrt_core and (sig == signal.SIGINT or sig == signal.SIGTERM)):
        global_aimrt_core.Shutdown()
        return

    sys.exit(0)


class ExampleServiceImpl(rpc_aimrt_rpc_pb2.ExampleService):
    def __init__(self, logger):
        super().__init__()
        self.logger = logger

    @staticmethod
    def PrintMetaInfo(logger, ctx_ref):
        meta_keys = ctx_ref.GetMetaKeys()
        for key in meta_keys:
            aimrt_py.info(logger, f"meta key: {key}, value: {ctx_ref.GetMetaValue(key)}")

    def GetFooData(self, ctx_ref, req):
        rsp = rpc_pb2.GetFooDataRsp()
        rsp.msg = "echo " + req.msg

        ExampleServiceImpl.PrintMetaInfo(self.logger, ctx_ref)
        aimrt_py.info(self.logger,
                      f"Server handle new rpc call. "
                      f"context: {ctx_ref.ToString()}, "
                      f"req: {MessageToJson(req)}, "
                      f"return rsp: {MessageToJson(rsp)}")

        return aimrt_py.RpcStatus(), rsp

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
    module_handle = aimrt_core.CreateModule("NormalRpcServerPymodule")

    # Register rpc service
    service = ExampleServiceImpl(module_handle.GetLogger())
    ret = module_handle.GetRpcHandle().RegisterService(service)
    assert ret, "Register service failed."

    # Start
    thread = threading.Thread(target=aimrt_core.Start)
    thread.start()

    while thread.is_alive():
        thread.join(1.0)

if __name__ == '__main__':
    main()
```

For RPC calls and processing based on ROS2 Srv protocol, the usage is essentially the same as protobuf-based RPC except for different data types.

Complete examples can be found at:
- {{ '[examples/py/ros2_rpc]({}/src/examples/py/ros2_rpc)'.format(code_site_root_path_url) }}
- {{ '[examples/py/pb_rpc]({}/src/examples/py/pb_rpc)'.format(code_site_root_path_url) }}