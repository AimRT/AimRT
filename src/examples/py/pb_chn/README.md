# protobuf channel examples


## protobuf channel http


一个基于 protobuf 协议与 http 后端的 python channel 示例，演示内容包括：
- 如何在 python 中使用 protobuf 协议作为 channel 传输协议；
- 如何基于 aimrt_py 创建模块的方式使用 Channel publish 和 subscribe 功能；
- 如何使用 http 类型的 channel 后端；


核心代码：
- [event.proto](../../../protocols/example/event.proto)
- [examples_py_pb_chn_publisher_app.py](./examples_py_pb_chn_publisher_app.py)
- [examples_py_pb_chn_subscriber_app.py](./examples_py_pb_chn_subscriber_app.py)


配置文件：
- [examples_py_pb_chn_http_pub_cfg.yaml](./cfg/examples_py_pb_chn_http_pub_cfg.yaml)
- [examples_py_pb_chn_http_sub_cfg.yaml](./cfg/examples_py_pb_chn_http_sub_cfg.yaml)



运行方式（linux）：
- [安装 `aimrt_py` 包](../../../../document/sphinx-cn/tutorials/quick_start/installation_py.md)；
- 运行本目录下的[build_examples_py_pb_chn.sh](./build_examples_py_pb_chn.sh)脚本，生成协议桩代码文件；
  - 如果本地没有 protoc 或者 protoc 版本小于 3.20，请安装或升级 protoc，或直接修改脚本中的 `protoc_cmd` 变量指向合适的路径；
- 运行本目录下的[start_examples_py_pb_chn_http_sub.sh](./start_examples_py_pb_chn_http_sub.sh)脚本，启动 subscriber；
- 在新终端里运行本目录下的[start_examples_py_pb_chn_http_pub.sh](./start_examples_py_pb_chn_http_pub.sh)脚本，启动 publisher 发布一条消息；
- 向 subscriber 进程所在终端里键入`ctrl-c`以停止进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalPublisherPyModule`：会在启动后向配置的 topic 中发布若干条 `ExampleEventMsg` 类型的消息，然后结束进程；
  - `NormalSubscriberPyModule`：会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息；
- 此示例使用 http 类型的 channel 后端进行通信，请确保相关端口未被占用；



## protobuf channel ros2

一个基于 protobuf 协议与 ros2 后端的 python channel 示例，演示内容包括：
- 如何在 python 中使用 protobuf 协议作为 channel 传输协议；
- 如何基于 aimrt_py 创建模块的方式使用 Channel publish 和 subscribe 功能；
- 如何使用 ros2 类型的 channel 后端；


核心代码：
- [event.proto](../../../protocols/example/event.proto)
- [examples_py_pb_chn_publisher_app.py](./examples_py_pb_chn_publisher_app.py)
- [examples_py_pb_chn_subscriber_app.py](./examples_py_pb_chn_subscriber_app.py)


配置文件：
- [examples_py_pb_chn_ros2_pub_cfg.yaml](./cfg/examples_py_pb_chn_ros2_pub_cfg.yaml)
- [examples_py_pb_chn_ros2_sub_cfg.yaml](./cfg/examples_py_pb_chn_ros2_sub_cfg.yaml)



运行方式（linux）：
- [安装 `aimrt_py` 包](../../../../document/sphinx-cn/tutorials/quick_start/installation_py.md)；
- 运行本目录下的[build_examples_py_pb_chn.sh](./build_examples_py_pb_chn.sh)脚本，生成协议桩代码文件；
  - 如果本地没有 protoc 或者 protoc 版本小于 3.20，请安装或升级 protoc，或直接修改脚本中的 `protoc_cmd` 变量指向合适的路径；
- 运行本目录下的[start_examples_py_pb_chn_ros2_sub.sh](./start_examples_py_pb_chn_ros2_sub.sh)脚本，启动 subscriber；
- 在新终端里运行本目录下的[start_examples_py_pb_chn_ros2_pub.sh](./start_examples_py_pb_chn_ros2_pub.sh)脚本，启动 publisher 发布一条消息；
- 向 subscriber 进程所在终端里键入`ctrl-c`以停止进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalPublisherPyModule`：会在启动后向配置的 topic 中发布若干条 `ExampleEventMsg` 类型的消息，然后结束进程；
  - `NormalSubscriberPyModule`：会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息；
- 此示例使用 ros2 类型的 channel 后端进行通信，请确保本地安装有 ROS2 Humble；


