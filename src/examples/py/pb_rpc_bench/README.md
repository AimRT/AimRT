# protobuf rpc benchmark

崩溃问题复现方法：

在 pb_rpc 目录下运行：

```
./build_examples_py_pb_rpc.sh
./start_examples_py_pb_rpc_http_server.sh
```

在 pb_rpc_bench 目录下运行：

```
./build_examples_py_pb_rpc_bench.sh
./start_benchmark_rpc_client.sh
```
