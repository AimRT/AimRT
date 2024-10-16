# protobuf channel benchmark


一个基于 protobuf 协议与 http 后端的 python channel benchmark 示例，演示内容包括：
- 如何在 python 中使用 protobuf 协议作为 channel 传输协议；
- 如何基于 aimrt_py 注册模块的方式使用 Channel publish 和 subscribe 功能；
- 如何使用 http 类型的 channel 后端；

核心代码：
- [benchmark.proto](../../../protocols/example/benchmark.proto)
- [benchmark_publisher_module.py](./benchmark_publisher_module.py)
- [benchmark_publisher_app.py](./benchmark_publisher_app.py)
- [benchmark_subscriber_module.py](./benchmark_subscriber_module.py)
- [benchmark_subscriber_app.py](./benchmark_subscriber_app.py)

配置文件：
- [benchmark_publisher_cfg.yaml](./cfg/benchmark_publisher_cfg.yaml)
- [benchmark_subscriber_cfg.yaml](./cfg/benchmark_subscriber_cfg.yaml)


运行方式（linux）：
- 安装 `aimrt_py` 包；
- 运行本目录下的[build_examples_py_pb_chn_bench.sh](./build_examples_py_pb_chn_bench.sh)脚本，生成协议桩代码文件；
  - 如果本地没有 protoc 或者 protoc 版本小于 3.20，请安装或升级 protoc，或直接修改脚本中的 `protoc_cmd` 变量指向合适的路径；
- 运行本目录下的[start_benchmark_subscriber.sh](./start_benchmark_subscriber.sh)脚本，启动 subscriber；
- 在新终端里运行本目录下的[start_benchmark_publisher.sh](./start_benchmark_publisher.sh)脚本，启动 publisher；
- Benchmark 运行结束后会输出 benchmark 结果并自动结束进程；


说明：
- 此示例创建了以下两个模块：
  - `BenchmarkPublisherModule`：会在启动后根据配置好的 bench plan 向指定的 test_topic_xx 中发布类型为 `BenchmarkMsg` 的消息，每个 bench plan 前后会发送一个 `BenchmarkSignal` 类型的消息通知 subscriber 当前的 benchmark 状态，所有 bench plan 结束后会发送一个 `BenchmarkSignal` 类型的消息通知 subscriber 当前的 benchmark 结束；
  - `BenchmarkSubscriberModule`：会订阅 channel 中的 test_topic_xx 消息，并统计接收到的消息的延迟分布；
- 此示例使用 http 类型的 channel 后端进行通信，请确保相关端口未被占用；
