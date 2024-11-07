# mqtt plugin examples

## protobuf rpc

一个基于 protobuf 协议、协程型接口与 mqtt 后端的 rpc 示例，演示内容包括：
- 如何在配置文件中加载**mqtt_plugin**；
- 如何使用 mqtt 类型的 rpc 后端；


核心代码：
- [rpc.proto](../../../protocols/example/rpc.proto)
- [normal_rpc_co_client_module.cc](../../cpp/pb_rpc/module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)
- [normal_rpc_co_server_module.cc](../../cpp/pb_rpc/module/normal_rpc_co_server_module/normal_rpc_co_server_module.cc)
- [service.cc](../../cpp/pb_rpc/module/normal_rpc_co_server_module/service.cc)


配置文件：
- [examples_plugins_mqtt_plugin_pb_rpc_client_cfg.yaml](./install/linux/bin/cfg/examples_plugins_mqtt_plugin_pb_rpc_client_cfg.yaml)
- [examples_plugins_mqtt_plugin_pb_rpc_server_cfg.yaml](./install/linux/bin/cfg/examples_plugins_mqtt_plugin_pb_rpc_server_cfg.yaml)



运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_MQTT_PLUGIN` 选项编译 AimRT；
- 在本地启动一个 mqtt broker，也可以使用其他 IP 地址的 mqtt broker，但需要修改示例配置中的 `broker_addr`； 
- 在终端运行 build 目录下`start_examples_plugins_mqtt_plugin_pb_rpc_server.sh`脚本启动服务端（srv 进程）；
- 开启新的终端运行 build 目录下`start_examples_plugins_mqtt_plugin_pb_rpc_client.sh`脚本启动客户端（cli 进程）；
- 分别在两个终端键入`ctrl-c`停止对应进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalRpcCoClientModule`：会基于 `work_thread_pool` 执行器，以配置的频率，通过协程 Client 接口，向 `ExampleService` 发起 RPC 请求；
  - `NormalRpcCoServerModule`：会注册 `ExampleService` 服务端，通过协程 Server 接口，提供 echo 功能；
- 此示例在 Rpc Client 端和 Server 端分别注册了两个 Filter 用于打印请求日志和计算耗时；
- 此示例将 `NormalRpcCoClientModule` 和 `NormalRpcCoServerModule` 分别集成到 `pb_rpc_client_pkg` 和 `pb_rpc_server_pkg` 两个 Pkg 中，并在两个配置文件中分别加载对应的 Pkg 到 srv 和 cli 进程中；
- 此示例加载了**mqtt_plugin**，并使用 mqtt 类型的 rpc 后端进行通信，配置 `tcp://127.0.0.1:1883` 作为 broker 地址，此外还在客户端配置了 `timeout_handle` 执行器作为超时执行器；

 

## ros2 rpc

一个基于 ros2 srv 协议、协程型接口与 mqtt 后端的 rpc 示例，演示内容包括：
- 如何在配置文件中加载**mqtt_plugin**；
- 如何使用 mqtt 类型的 rpc 后端；


核心代码：
- [example_ros2/srv/RosTestRpc.srv](../../../protocols/example_ros2/srv/RosTestRpc.srv)
- [normal_rpc_co_client_module.cc](../../cpp/ros2_rpc/module/normal_rpc_co_client_module/normal_rpc_co_client_module.cc)
- [normal_rpc_co_server_module.cc](../../cpp/ros2_rpc/module/normal_rpc_co_server_module/normal_rpc_co_server_module.cc)
- [service.cc](../../cpp/ros2_rpc/module/normal_rpc_co_server_module/service.cc)


配置文件：
- [examples_plugins_mqtt_plugin_ros2_rpc_server_cfg.yaml](./install/linux/bin/cfg/examples_plugins_mqtt_plugin_ros2_rpc_server_cfg.yaml)
- [examples_plugins_mqtt_plugin_ros2_rpc_client_cfg.yaml](./install/linux/bin/cfg/examples_plugins_mqtt_plugin_ros2_rpc_client_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_MQTT_PLUGIN` 选项编译 AimRT；
- 在本地启动一个 mqtt broker，也可以使用其他 IP 地址的 mqtt broker，但需要修改示例配置中的 `broker_addr`； 
- 在终端运行 build 目录下`start_examples_plugins_mqtt_plugin_ros2_rpc_server.sh`脚本启动服务端（srv进程）；
- 开启新的终端运行 build 目录下`start_examples_plugins_mqtt_plugin_ros2_rpc_client.sh`脚本启动客户端（cli进程）；
- 分别在两个终端键入`ctrl-c`停止对应进程；


说明：
- 本示例与 **protobuf rpc** 示例基本一致，除了业务层使用的是 ros2 srv 形式的协议；



## protobuf channel

一个基于 protobuf 协议与 mqtt 后端的 channel 示例，演示内容包括：
- 如何在配置文件中加载**mqtt_plugin**；
- 如何使用 mqtt 类型的 channel 后端；



核心代码：
- [event.proto](../../../protocols/example/event.proto)
- [normal_publisher_module.cc](../../cpp/pb_chn/module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](../../cpp/pb_chn/module/normal_subscriber_module/normal_subscriber_module.cc)


配置文件：
- [examples_plugins_mqtt_plugin_pb_chn_pub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_mqtt_plugin_pb_chn_pub_cfg.yaml)
- [examples_plugins_mqtt_plugin_pb_chn_sub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_mqtt_plugin_pb_chn_sub_cfg.yaml)

运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_MQTT_PLUGIN` 选项编译 AimRT；
- 在本地启动一个 mqtt broker，也可以使用其他 IP 地址的 mqtt broker，但需要修改示例配置中的 `broker_addr`； 
- 在终端运行 build 目录下`start_examples_plugins_mqtt_plugin_pb_chn_sub.sh`脚本启动订阅端（sub进程）；
- 再开启一个新的终端窗口运行`start_examples_plugins_mqtt_plugin_pb_chn_pub.sh`脚本启动发布端（pub进程）；
- 分别在两个终端键入`ctrl-c`停止对应进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalPublisherModule`：会基于 `work_thread_pool` 执行器，以配置的频率、向配置的 topic 中发布 `ExampleEventMsg` 类型的消息；
  - `NormalSubscriberModule`：会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息；
- 此示例将 `NormalPublisherModule` 和 `NormalSubscriberModule` 分别集成到 `pb_chn_pub_pkg` 和 `pb_chn_sub_pkg` 两个 Pkg 中，并在两个配置文件中分别加载对应的 Pkg 到 pub 和 sub 进程中；
- 此示例加载了**mqtt_plugin**，并使用 mqtt 类型的 channel 后端进行通信，配置 `tcp://127.0.0.1:1883` 作为 broker 地址；


## ros2 channel

一个基于 ros2 msg 协议与 mqtt 后端的 channel 示例，演示内容包括：
- 如何在配置文件中加载**mqtt_plugin**；
- 如何使用 mqtt 类型的 channel 后端；



核心代码：
- [example_ros2/msg/RosTestMsg.msg](../../../protocols/example_ros2/msg/RosTestMsg.msg)
- [normal_publisher_module.cc](../../cpp/ros2_chn/module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](../../cpp/ros2_chn/module/normal_subscriber_module/normal_subscriber_module.cc)


配置文件：
- [examples_plugins_mqtt_plugin_ros2_chn_pub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_mqtt_plugin_ros2_chn_pub_cfg.yaml)
- [examples_plugins_mqtt_plugin_ros2_chn_sub_cfg.yaml](./install/linux/bin/cfg/examples_plugins_mqtt_plugin_ros2_chn_sub_cfg.yaml)


运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_MQTT_PLUGIN` 选项编译 AimRT；
- 在本地启动一个 mqtt broker，也可以使用其他 IP 地址的 mqtt broker，但需要修改示例配置中的 `broker_addr`； 
- 在终端运行 build 目录下`start_examples_plugins_mqtt_plugin_ros2_chn_sub.sh`脚本启动订阅端（sub进程）；
- 再开启一个新的终端窗口运行`start_examples_plugins_mqtt_plugin_ros2_chn_pub.sh`脚本启动发布端（pub进程）；
- 分别在两个终端键入`ctrl-c`停止对应进程；


说明：
- 本示例与 **protobuf rpc** 示例基本一致，除了业务层使用的是 ros2 msg 形式的协议；


## protobuf channel with SSL/TLS

一个基于 protobuf 协议与 mqtt 后端的 channel 示例，演示内容包括：
- 如何在配置文件中加载**mqtt_plugin**；
- 如何使用 mqtt 类型的 channel 后端；
- 如何配置 SSL/TLS 加密；



核心代码：
- [event.proto](../../../protocols/example/event.proto)
- [normal_publisher_module.cc](../../cpp/pb_chn/module/normal_publisher_module/normal_publisher_module.cc)
- [normal_subscriber_module.cc](../../cpp/pb_chn/module/normal_subscriber_module/normal_subscriber_module.cc)


配置文件：
- [examples_plugins_mqtt_plugin_pb_chn_pub_with_ssl_cfg.yaml](./install/linux/bin/cfg/examples_plugins_mqtt_plugin_pb_chn_pub_with_ssl_cfg.yaml)
- [examples_plugins_mqtt_plugin_pb_chn_sub_with_ssl_cfg.yaml](./install/linux/bin/cfg/examples_plugins_mqtt_plugin_pb_chn_sub_with_ssl_cfg.yaml)

运行方式（linux）：
- 开启 `AIMRT_BUILD_EXAMPLES`、`AIMRT_BUILD_MQTT_PLUGIN` 选项编译 AimRT；
- 生成加密通信所需的证书和密钥文件（主要需要的是ca_crt.pem、server_crt.pem、server_key.pem、client_crt.pem、client_key.pem 这五个文件），以下是一个简单的生成示例：
  
  ```shell
  # 生成 CA 私钥
  openssl genpkey -algorithm RSA -out ca_key.pem

  # 生成 CA 自签名证书
  openssl req -x509 -new -key ca_key.pem -sha256 -days 3650 -out ca_crt.pem

  ```

  ```shell
  # 生成服务器私钥
  openssl genpkey -algorithm RSA -out server_key.pem

  # 生成服务器证书签名请求 (CSR)
  openssl req -new -key server_key.pem -out server_csr.pem

  # 使用 CA 签署服务器证书
  openssl x509 -req -in server_csr.pem -CA ca_crt.pem -CAkey ca_key.pem -CAcreateserial -out server_crt.pem -days 365 -sha256
  ```

  ```shell
  # 生成客户端私钥
  openssl genpkey -algorithm RSA -out client_key.pem

  # 生成客户端证书签名请求 (CSR)
  openssl req -new -key client_key.pem -out client_csr.pem

  # 使用 CA 签署客户端证书
  openssl x509 -req -in client_csr.pem -CA ca_crt.pem -CAkey ca_key.pem -CAcreateserial -out client_crt.pem -days 365 -sha256
  ```


  ````shell
  # [可选] 使用 OpenSSL 的 openssl pkcs8 命令将 client 的私钥加密
  openssl pkcs8 -topk8 -inform PEM -outform PEM -in client_key.pem -out client_key_encrypted.pem -v2 aes-256-cbc
  ````

- 将 `ca_crt.pem`、`server_crt.pem`、`server_key.pem`的路径复制到 broker 配置文件中对应的位置， 并配置需要单向认证/双向认证， 以及地址（默认 0.0.0.0:8883）；
- 将 `ca_crt.pem`、`client_crt.pem`、`client_key.pem`的路径依次复制到客户端配置文件中对应的`truststore`、`client_cert`、`client_key`， 如果设置客户端私钥文件被加密，则将设置的密码配置在`client_key_password`中；
- 在本地启动一个 mqtt broker，也可以使用其他 IP 地址的 mqtt broker，但需要修改示例配置中的 `broker_addr`； 
- 在终端运行 build 目录下`start_examples_plugins_mqtt_plugin_pb_chn_sub_with_ssl.sh`脚本启动订阅端（sub进程）；
- 再开启一个新的终端窗口运行`start_examples_plugins_mqtt_plugin_pb_chn_pub_with_ssl.sh`脚本启动发布端（pub进程）；
- 分别在两个终端键入`ctrl-c`停止对应进程；


说明：
- 此示例创建了以下两个模块：
  - `NormalPublisherModule`：会基于 `work_thread_pool` 执行器，以配置的频率、向配置的 topic 中发布 `ExampleEventMsg` 类型的消息；
  - `NormalSubscriberModule`：会订阅配置的 topic 下的 `ExampleEventMsg` 类型的消息；
- 此示例将 `NormalPublisherModule` 和 `NormalSubscriberModule` 分别集成到 `pb_chn_pub_pkg` 和 `pb_chn_sub_pkg` 两个 Pkg 中，并在两个配置文件中分别加载对应的 Pkg 到 pub 和 sub 进程中；
- 此示例加载了**mqtt_plugin**，并使用 mqtt 类型的 channel 后端进行通信，配置 `ssl://127.0.0.1:8883` 作为 broker 地址， 默认该端口用于 SSL/TLS 加密通信；

- 直接运行给定的示例可能不能正常工作，需要修改证书和密钥的路径。