# Rpc


## 相关链接

参考示例：
- {{ '[examples_py_pb_rpc_client_app.py]({}/src/examples/py/pb_rpc/examples_py_pb_rpc_client_app.py)'.format(code_site_root_path_url) }}
- {{ '[examples_py_pb_rpc_server_app.py]({}/src/examples/py/pb_rpc/examples_py_pb_rpc_server_app.py)'.format(code_site_root_path_url) }}

## protobuf 协议


协议用于确定 RPC 中客户端和服务端的消息格式。一般来说，协议都是使用一种与具体的编程语言无关的 IDL ( Interface description language )描述，然后由某种工具转换为各个语言的代码。对于 RPC 来说，这里需要两个步骤：
- 参考[Channel](./channel.md)章节中的介绍，开发者需要先利用一些官方的工具为协议文件中的**消息类型**生成指定编程语言中的代码；
- 开发者需要使用 AimRT 提供的工具，为协议文件中**服务定义**生成指定编程语言中的代码；


[Protobuf](https://protobuf.dev/)是一种由 Google 开发的、用于序列化结构化数据的轻量级、高效的数据交换格式，是一种广泛使用的 IDL。它不仅能够描述消息结构，还提供了`service`语句来定义 RPC 服务。

当前版本 AimRT Python 只支持 protobuf 协议。在使用 AimRT Python 发起/处理 RPC 请求之前，使用者需要先基于 protobuf 协议生成一些 python 的桩代码。


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

然后使用 Protobuf 官方提供的 protoc 工具进行转换，生成消息结构部分的 Python 代码，例如：
```shell
protoc --python_out=. rpc.proto
```

这将生成`rpc_pb2.py`文件，包含了根据定义的消息类型生成的 Python 接口。


在这之后，还需要使用 AimRT 提供的 protoc 插件，生成服务定义部分的 Python 桩代码，例如：
```shell
protoc --aimrt_rpc_out=. --plugin=protoc-gen-aimrt_rpc=./protoc_plugin_py_gen_aimrt_py_rpc.py rpc.proto
```

这将生成`rpc_aimrt_rpc_pb2.py`文件，包含了根据定义的服务生成的 Python 接口，我们的业务代码中需要 import 此文件。


## RpcHandle

模块可以通过调用`CoreRef`句柄的`GetRpcHandle()`接口，获取`RpcHandleRef`句柄。一般情况下，开发者不会直接使用`RpcHandleRef`直接提供的接口，而是根据 RPC IDL 文件生成一些桩代码，对`RpcHandleRef`句柄做一些封装，然后在业务代码中使用这些经过封装的接口。


这些经过封装的接口的具体形式将在本文档后续章节介绍。开发者在使用 RPC 功能时需要按照以下步骤使用这些接口：
- Client 端：
  - 在`Initialize`阶段，调用**注册 RPC Client 方法**的接口；
  - 在`Start`阶段，调用 **RPC Invoke** 的接口，以实现 RPC 调用；
- Server 端：
  - 在`Initialize`阶段，**注册 RPC Server 服务**的接口；


## RpcStatus

在 RPC 调用或者 RPC 处理时，使用者可以通过一个`RpcStatus`类型的变量获取 RPC 过程中的错误情况，其包含的接口如下：
- `OK()->bool` ：是否成功；
- `Code()->int` ：错误码；
- `ToString()->str` ：转字符串；


`RpcStatus`类型非常轻量，其中只包含一个错误码字段。使用者可以通过构造函数或 Set 方法设置这个 Code，可以通过 Get 方法获取这个 Code。错误码的枚举值可以参考{{ '[rpc_status_base.h]({}/src/interface/aimrt_module_c_interface/rpc/rpc_status_base.h)'.format(code_site_root_path_url) }}文件中的定义。


请注意，`RpcStatus`中的错误信息一般仅表示框架层面的错误，例如服务未找到、网络错误或者序列化错误等，供开发者排查框架层面的问题。如果开发者需要返回业务层面的错误，建议在业务包中添加相应的字段。


## Client


在 AimRT Python RPC 桩代码工具生成的代码里，如`xxx_aimrt_rpc_pb2.py`文件里，提供了`XXXProxy`类型，开发者基于此 Proxy 接口来发起 RPC 调用。此接口是同步型接口，使用此 Proxy 接口发起 RPC 调用后会阻塞当前线程，直到收到回包或请求超时。


使用该 Proxy 发起 RPC 调用非常简单，一般分为以下几个步骤：
- **Step 0**：引用根据 protobuf 协议生成的桩代码包，例如`xxx_aimrt_rpc_pb2.py`；
- **Step 1**：在`Initialize`阶段调用该 Proxy 的`RegisterClientFunc`静态方法注册 RPC Client；
- **Step 2**：在`Start`阶段里某个业务函数里发起 RPC 调用：
  - **Step 2-1**：创建一个 Proxy 实例，构造参数是`RpcHandleRef`；
  - **Step 2-2**：创建 Req，并填充 Req 内容；
  - **Step 2-3**：【可选】创建 ctx，设置超时等信息；
  - **Step 2-4**：基于 proxy，传入 ctx、Req，发起 RPC 调用，同步等待 RPC 调用结束，保证在整个调用周期里 ctx、Req 都保持有效且不会改动，最终获取返回的 status 和 Rsp；
  - **Step 2-5**：解析 status 和 Rsp；



以下是一个使用 AimRT Python 进行 RPC Client 调用的示例，通过 Create Module 方式拿到`CoreRef`句柄。如果是基于`Module`模式在`Initialize`方法中拿到`CoreRef`句柄，使用方式也类似：

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
    status, rsp = proxy.GetFooData(ctx, req)

    aimrt_py.info(module_handle.GetLogger(),
                  "Call rpc done, status: {}, req: {}, rsp: {}"
                  .format(status.ToString(), MessageToJson(req), MessageToJson(rsp)))

    # Shutdown
    aimrt_core.Shutdown()

    thread.join()

if __name__ == '__main__':
    main()
```


## Server

在 AimRT Python RPC 桩代码工具生成的代码里，如`xxx_aimrt_rpc_pb2.py`文件里，提供了一个继承了`aimrt_py.ServiceBase`的 Service 基类，开发者需要继承该 Service 基类并实现其中的虚接口。此 Service 接口是同步型接口，开发者只能在 handle 中阻塞的完成所有操作并在最后返回回包。

使用该接口提供 RPC 服务，一般分为以下几个步骤：
- **Step 0**：引用根据 protobuf 协议生成的桩代码包，例如`xxx_aimrt_rpc_pb2.py`；
- **Step 1**：开发者实现一个 Impl 类，继承包中的`XXXService`，并实现其中的虚接口，接口形式为`(ctx, req)->status, rsp`；
  - **Step 1-1**：解析 Ctx 和 Req，并填充 Rsp；
  - **Step 1-2**：返回`RpcStatus`和 Rsp；
- **Step 2**：在`Initialize`阶段调用`RpcHandleRef`的`RegisterService`方法注册 RPC Service；


以下是一个使用 AimRT Python 进行 RPC Service 处理的示例，通过 Create Module 方式拿到`CoreRef`句柄。如果是基于`Module`模式在`Initialize`方法中拿到`CoreRef`句柄，使用方式也类似：

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
    def __init__(self):
        super().__init__()

    def GetFooData(self, ctx_ref, req):
        rsp = rpc_pb2.GetFooDataRsp()
        rsp.msg = "echo " + req.msg

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
    service = ExampleServiceImpl()
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
