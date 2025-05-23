# Rpc

## Related Links

Reference examples:
- {{ '[examples_py_pb_rpc_client_app.py]({}/src/examples/py/pb_rpc/examples_py_pb_rpc_client_app.py)'.format(code_site_root_path_url) }}
- {{ '[examples_py_pb_rpc_server_app.py]({}/src/examples/py/pb_rpc/examples_py_pb_rpc_server_app.py)'.format(code_site_root_path_url) }}
- {{ '[examples_py_ros2_rpc_client_app.py]({}/src/examples/py/ros2_rpc/examples_py_ros2_rpc_client_app.py)'.format(code_site_root_path_url) }}
- {{ '[examples_py_ros2_rpc_server_app.py]({}/src/examples/py/ros2_rpc/examples_py_ros2_rpc_server_app.py)'.format(code_site_root_path_url) }}

## Protocol

Protocols are used to define the message format between RPC clients and servers. Typically, protocols are described using an Interface Description Language (IDL) that is programming language agnostic, then converted into language-specific code through tools. For RPC, this involves two steps:
- As introduced in the [Channel](./channel.md) section, developers first need to use official tools to generate code for **message types** defined in the protocol file for their target programming language;
- Developers then need to use tools provided by AimRT to generate code for **service definitions** in the protocol file for their target programming language;

### Protobuf

[Protobuf](https://protobuf.dev/) is a lightweight, efficient data interchange format developed by Google for serializing structured data. It's a widely used IDL that can describe message structures and also provides the `service` statement to define RPC services.

When using Protobuf, developers first define a `.proto` file containing message structures and RPC services. For example, `rpc.proto`:

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

Then use the official protoc tool from Protobuf to generate Python code for the message structures:
```shell
protoc --python_out=. rpc.proto
```

This generates the `rpc_pb2.py` file containing Python interfaces for the defined message types.

After this, developers need to use the protoc plugin provided by AimRT to generate Python stub code for service definitions:
```shell
protoc --aimrt_rpc_out=. --plugin=protoc-gen-aimrt_rpc=./protoc_plugin_py_gen_aimrt_py_rpc.py rpc.proto
```

This generates the `rpc_aimrt_rpc_pb2.py` file containing Python interfaces for the defined services, which needs to be imported in business code.

### ROS2 Srv

ROS2 Srv is a format used for RPC definitions in ROS2. When using it, developers first define a ROS2 Package containing a `.srv` file, such as `example.srv`:

```
byte[]  data
---
int64   code
```

Where `---` separates the Req (Request) and Rsp (Response) definitions.

The aimrt_py installation automatically includes a command-line tool `aimrt_py-gen-ros2-rpc` for generating AimRT Python RPC stub code from ROS2 Srv files.

```shell
aimrt_py-gen-ros2-rpc --pkg_name=example_pkg --srv_file=./example.srv --output_path=./
```

Where pkg_name is the ROS2 Package name, srv_file is the path to the ROS2 Srv file, and output_path is the destination for generated stub code. This produces an `example_aimrt_rpc_ros2.py` file containing Python interfaces for the defined services, which needs to be imported in business code.

Note that `aimrt_py-gen-ros2-rpc` only generates Req and Rsp definitions, not message structure code. Developers still need to generate message structure code separately (by building the ROS2 Package and generating message code, see [ROS2 official documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) for details).

## RpcHandle

Modules can obtain an `RpcHandleRef` handle by calling the `GetRpcHandle()` interface of the `CoreRef` handle. Typically, developers don't use the raw interfaces provided by `RpcHandleRef` directly. Instead, they use stub code generated from RPC IDL files that wraps the `RpcHandleRef` handle, then work with these wrapped interfaces in business code.

The specific forms of these wrapped interfaces will be introduced in later sections. When using RPC functionality, developers should follow these steps:
- Client side:
  - During `Initialize` phase, call the **RPC Client registration method** interface;
  - During `Start` phase, call the **RPC Invoke** interface to make RPC calls;
- Server side:
  - During `Initialize` phase, call the **RPC Server registration** interface;

## RpcStatus

During RPC calls or processing, users can check for errors through an `RpcStatus` variable, which provides these interfaces:
- `OK()->bool`: Whether the operation succeeded;
- `Code()->int`: Error code;
- `ToString()->str`: Convert to string;

The `RpcStatus` type is very lightweight, containing only an error code field. Users can set this code via constructors or Set methods, and retrieve it via Get methods. For error code enumerations, refer to {{ '[rpc_status_base.h]({}/src/interface/aimrt_module_c_interface/rpc/rpc_status_base.h)'.format(code_site_root_path_url) }}.

Note that `RpcStatus` errors typically indicate framework-level issues like service not found, network errors, or serialization problems, helping developers troubleshoot framework issues. For business-level errors, developers should add appropriate fields in their business packages.## RpcContext

RpcContext is the context information during RPC calls, allowing developers to set certain contextual details such as timeout duration and metadata. The specific interfaces are as follows:
- `CheckUsed()->bool`: Checks whether the Context has been used;
- `SetUsed()->None`: Marks the Context as used;
- `Reset()->None`: Resets the Context;
- `GetType()->aimrt_rpc_context_type_t`: Retrieves the Context type;
- `Timeout()->datetime.timedelta`: Gets the timeout duration;
- `SetTimeout(timeout: datetime.timedelta)->None`: Sets the timeout duration;
- `SetMetaValue(key: str, value: str)->None`: Sets metadata;
- `GetMetaValue(key: str)->str`: Retrieves metadata;
- `GetMetaKeys()->List[str]`: Gets the list of all keys in the metadata key-value pairs;
- `SetToAddr(addr: str)->None`: Sets the target address;
- `GetToAddr()->str`: Retrieves the target address;
- `SetSerializationType(serialization_type: str)->None`: Sets the serialization type;
- `GetSerializationType()->str`: Retrieves the serialization type;
- `GetFunctionName()->str`: Gets the function name;
- `SetFunctionName(func_name: str)->None`: Sets the function name;
- `ToString()->str`: Retrieves context information, returning a human-readable string representation;

`RpcContextRef` is the reference type of `RpcContext`. Except for lacking the `Reset` interface, all other interfaces are identical to `RpcContext`.

`aimrt_rpc_context_type_t` is an enumeration type defining the context type, with possible values being `AIMRT_RPC_CLIENT_CONTEXT` or `AIMRT_RPC_SERVER_CONTEXT`, indicating whether it is a client-side or server-side context.

## Client

In the stub code generated by the AimRT Python RPC stub code tool, such as in the `xxx_aimrt_rpc_pb2.py` file, the `XXXProxy` type is provided. Developers use this Proxy interface to initiate RPC calls. This is a synchronous interface; invoking an RPC call through this Proxy will block the current thread until a response is received or the request times out.

Using this Proxy to initiate RPC calls is straightforward and generally involves the following steps:
- **Step 0**: Import the stub code package generated from the protobuf protocol, e.g., `xxx_aimrt_rpc_pb2.py`;
- **Step 1**: During the `Initialize` phase, call the `RegisterClientFunc` static method of the Proxy to register the RPC Client;
- **Step 2**: In a business function within the `Start` phase, initiate the RPC call:
  - **Step 2-1**: Create a Proxy instance, with the constructor parameter being `RpcHandleRef`;
  - **Step 2-2**: Create a Req and populate its content;
  - **Step 2-3**: [Optional] Create a ctx and set timeout or other information;
  - **Step 2-4**: Use the proxy to initiate the RPC call, passing in ctx and Req. Synchronously wait for the RPC call to complete, ensuring that ctx and Req remain valid and unmodified throughout the call. Finally, retrieve the returned status and Rsp;
  - **Step 2-5**: Parse the status and Rsp;

Below is an example of using AimRT Python to perform an RPC Client call based on the protobuf protocol, obtaining the `CoreRef` handle via the Create Module approach. If the `CoreRef` handle is obtained in the `Initialize` method under `Module` mode, the usage is similar:

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
# Server

In the stub code generated by the AimRT Python RPC tool, such as the `xxx_aimrt_rpc_pb2.py` file, a Service base class that inherits from `aimrt_py.ServiceBase` is provided. Developers need to inherit this Service base class and implement its virtual interfaces. This Service interface is synchronous, meaning developers must complete all operations in a blocking manner within the handle and return the response at the end.

To provide an RPC service using this interface, the following steps are generally followed:
- **Step 0**: Reference the stub code package generated from the protobuf protocol, such as `xxx_aimrt_rpc_pb2.py`;
- **Step 1**: Developers implement an Impl class that inherits the `XXXService` from the package and implements its virtual interfaces, with the interface format as `(ctx, req)->status, rsp`;
  - **Step 1-1**: Parse Ctx and Req, and populate Rsp;
  - **Step 1-2**: Return `RpcStatus` and Rsp;
- **Step 2**: During the `Initialize` phase, call the `RegisterService` method of `RpcHandleRef` to register the RPC Service;

Here is an example of using AimRT Python to handle RPC Service based on the protobuf protocol, obtaining the `CoreRef` handle via the Create Module approach. If the `CoreRef` handle is obtained in the `Initialize` method under the `Module` mode, the usage is similar:

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

For RPC calls and handling based on the ROS2 Srv protocol, apart from the difference in data types, the usage is largely the same as that based on the protobuf protocol.

Complete examples can be found at:
- {{ '[examples/py/ros2_rpc]({}/src/examples/py/ros2_rpc)'.format(code_site_root_path_url) }}
- {{ '[examples/py/pb_rpc]({}/src/examples/py/pb_rpc)'.format(code_site_root_path_url) }}