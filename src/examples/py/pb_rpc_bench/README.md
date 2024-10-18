# protobuf rpc benchmark


一个基于 protobuf 协议与 http 后端的 python rpc 基准测试示例，演示内容包括：
- 如何在 python 中使用 protobuf 协议作为 rpc 传输协议；
- 如何基于 aimrt_py 注册模块的方式使用 rpc client 和 server 功能；
- 如何使用 http 类型的 rpc 后端；
- 如何使用 executor 功能；


核心代码：
- [rpc.proto](../../../protocols/example/rpc.proto)
- [benchmark_rpc_client_module.py](./benchmark_rpc_client_module.py)
- [benchmark_rpc_client_app.py](./benchmark_rpc_client_app.py)
- [examples_py_pb_rpc_http_server_app.py](./examples_py_pb_rpc_http_server_app.py)


配置文件：
- [benchmark_rpc_client_cfg.yaml](./cfg/benchmark_rpc_client_cfg.yaml)
- [examples_py_pb_rpc_http_server_cfg.yaml](./cfg/examples_py_pb_rpc_http_server_cfg.yaml)

运行方式（linux）：
- [安装 `aimrt_py` 包](../../../../document/sphinx-cn/tutorials/quick_start/installation_py.md)；
- 运行本目录下的[build_examples_py_pb_rpc_bench.sh](./build_examples_py_pb_rpc_bench.sh)脚本，生成协议桩代码文件；
  - 如果本地没有 protoc 或者 protoc 版本小于 3.20，请安装或升级 protoc，或直接修改脚本中的 `protoc_cmd` 变量指向合适的路径；
- 运行本目录下的[start_examples_py_pb_rpc_http_server.sh](./start_examples_py_pb_rpc_http_server.sh)脚本，启动 RPC Server；
- 在另一个终端里运行本目录下的[start_benchmark_rpc_client.sh](./start_benchmark_rpc_client.sh)脚本，启动 RPC Client；
- 向 RPC Client 和 RPC Server 进程所在终端里键入`ctrl-c`以停止进程；


说明：
- 此示例创建了以下两个模块：
  - `BenchmarkRpcClientModule`：会在启动后向 `ExampleService` 发起基准测试请求，并输出测试结果；
  - `NormalRpcServerPyModule`：会注册 `ExampleService` 服务端，通过 Server 接口，提供 echo 功能；
- 此示例使用 http 类型的 rpc 后端进行通信，请确保相关端口未被占用；
