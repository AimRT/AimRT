# protobuf rpc examples


## protobuf rpc http


一个基于 protobuf 协议与 http 后端的 python rpc 示例，演示内容包括：
- 如何在 python 中使用 protobuf 协议作为 rpc 传输协议；
- 如何基于 aimrt_py 创建模块的方式使用 rpc client 和 server 功能；
- 如何使用 http 类型的 rpc 后端；


核心代码：
- [rpc.proto](../../../protocols/example/rpc.proto)
- [examples_py_pb_rpc_client_app.py](./examples_py_pb_rpc_client_app.py)
- [examples_py_pb_rpc_server_app.py](./examples_py_pb_rpc_server_app.py)


配置文件：
- [examples_py_pb_rpc_http_client_cfg.yaml](./cfg/examples_py_pb_rpc_http_client_cfg.yaml)
- [examples_py_pb_rpc_http_server_cfg.yaml](./cfg/examples_py_pb_rpc_http_server_cfg.yaml)



运行方式（linux）：
- [安装 `aimrt_py` 包](../../../../document/sphinx-cn/tutorials/quick_start/installation_py.md)；
- 运行本目录下的[build_examples_py_pb_rpc.sh](./build_examples_py_pb_rpc.sh)脚本，生成协议桩代码文件；
  - 如果本地没有 protoc 或者 protoc 版本小于 3.20，请安装或升级 protoc，或直接修改脚本中的 `protoc_cmd` 变量指向合适的路径；
- 运行本目录下的[start_examples_py_pb_rpc_http_server.sh](./start_examples_py_pb_rpc_http_server.sh)脚本，启动 RPC Server；
- 在新终端里运行本目录下的[start_examples_py_pb_rpc_http_client.sh](./start_examples_py_pb_rpc_http_client.sh)脚本，启动 Rpc client 发起一次请求；
- 向 RPC Server 进程所在终端里键入`ctrl-c`以停止进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalRpcClientPyModule`：会在启动后向 `ExampleService` 发起一次 `GetFooData` 的 RPC 请求，等收到回包并解析后结束进程；
  - `NormalRpcServerPymodule`：会注册 `ExampleService` 服务端，通过 Server 接口，提供 echo 功能；
- 此示例使用 http 类型的 rpc 后端进行通信，请确保相关端口未被占用；



## protobuf rpc grpc


一个基于 protobuf 协议与 grpc 后端的 python rpc 示例，演示内容包括：
- 如何在 python 中使用 protobuf 协议作为 rpc 传输协议；
- 如何基于 aimrt_py 创建模块的方式使用 rpc client 和 server 功能；
- 如何使用 grpc 类型的 rpc 后端；


核心代码：
- [rpc.proto](../../../protocols/example/rpc.proto)
- [examples_py_pb_rpc_client_app.py](./examples_py_pb_rpc_client_app.py)
- [examples_py_pb_rpc_server_app.py](./examples_py_pb_rpc_server_app.py)


配置文件：
- [examples_py_pb_rpc_grpc_client_cfg.yaml](./cfg/examples_py_pb_rpc_grpc_client_cfg.yaml)
- [examples_py_pb_rpc_grpc_server_cfg.yaml](./cfg/examples_py_pb_rpc_grpc_server_cfg.yaml)



运行方式（linux）：
- [安装 `aimrt_py` 包](../../../../document/sphinx-cn/tutorials/quick_start/installation_py.md)；
- 运行本目录下的[build_examples_py_pb_rpc.sh](./build_examples_py_pb_rpc.sh)脚本，生成协议桩代码文件；
  - 如果本地没有 protoc 或者 protoc 版本小于 3.20，请安装或升级 protoc，或直接修改脚本中的 `protoc_cmd` 变量指向合适的路径；
- 运行本目录下的[start_examples_py_pb_rpc_grpc_server.sh](./start_examples_py_pb_rpc_grpc_server.sh)脚本，启动 RPC Server；
- 在新终端里运行本目录下的[start_examples_py_pb_rpc_grpc_client.sh](./start_examples_py_pb_rpc_grpc_client.sh)脚本，启动 Rpc client 发起一次请求；
- 向 RPC Server 进程所在终端里键入`ctrl-c`以停止进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalRpcClientPyModule`：会在启动后向 `ExampleService` 发起一次 `GetFooData` 的 RPC 请求，等收到回包并解析后结束进程；
  - `NormalRpcServerPymodule`：会注册 `ExampleService` 服务端，通过 Server 接口，提供 echo 功能；
- 此示例使用 grpc 类型的 rpc 后端进行通信，请确保相关端口未被占用；



## protobuf rpc ros2



一个基于 protobuf 协议与 ros2 后端的 python rpc 示例，演示内容包括：
- 如何在 python 中使用 protobuf 协议作为 rpc 传输协议；
- 如何基于 aimrt_py 创建模块的方式使用 rpc client 和 server 功能；
- 如何使用 ros2 类型的 rpc 后端；


核心代码：
- [rpc.proto](../../../protocols/example/rpc.proto)
- [examples_py_pb_rpc_client_app.py](./examples_py_pb_rpc_client_app.py)
- [examples_py_pb_rpc_server_app.py](./examples_py_pb_rpc_server_app.py)


配置文件：
- [examples_py_pb_rpc_ros2_client_cfg.yaml](./cfg/examples_py_pb_rpc_ros2_client_cfg.yaml)
- [examples_py_pb_rpc_ros2_server_cfg.yaml](./cfg/examples_py_pb_rpc_ros2_server_cfg.yaml)



运行方式（linux）：
- [安装 `aimrt_py` 包](../../../../document/sphinx-cn/tutorials/quick_start/installation_py.md)；
- 运行本目录下的[build_examples_py_pb_rpc.sh](./build_examples_py_pb_rpc.sh)脚本，生成协议桩代码文件；
  - 如果本地没有 protoc 或者 protoc 版本小于 3.20，请安装或升级 protoc，或直接修改脚本中的 `protoc_cmd` 变量指向合适的路径；
- 运行本目录下的[start_examples_py_pb_rpc_ros2_server.sh](./start_examples_py_pb_rpc_ros2_server.sh)脚本，启动 RPC Server；
- 在新终端里运行本目录下的[start_examples_py_pb_rpc_ros2_client.sh](./start_examples_py_pb_rpc_ros2_client.sh)脚本，启动 Rpc client 发起一次请求；
- 向 RPC Server 进程所在终端里键入`ctrl-c`以停止进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalRpcClientPyModule`：会在启动后向 `ExampleService` 发起一次 `GetFooData` 的 RPC 请求，等收到回包并解析后结束进程；
  - `NormalRpcServerPymodule`：会注册 `ExampleService` 服务端，通过 Server 接口，提供 echo 功能；
- 此示例使用 ros2 类型的 rpc 后端进行通信，请确保本地安装有 ROS2 Humble；
