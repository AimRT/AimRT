
# AimRT 中的协议

协议在 AimRT 中的主要作用是实现组件之间的数据交换和通信。无论是在同一进程内还是跨进程通信，AimRT都提供了一个统一的接口和数据序列化/反序列化机制，确保数据能够正确传输和解析。AimRT 现支持 ros2 和 protobuf 两种协议：

参考 AimRT 源码中的 `src/protococols` 目录

```
src/protocols
├── pb
│   ├── actuator
│   ├── common
│   ├── example
│   ├── geometry
│   └── sensor
├── plugins
│   ├── log_control_plugin
│   ├── parameter_plugin
│   ├── record_playback_plugin
│   ├── ros2_plugin_proto
│   ├── time_manipulator_plugin
│   └── topic_logger_plugin
└── ros2
    ├── aimrt_msgs
    ├── example_ros2
    └── ros2_msgs
```

## type_support_pkg 概念
type_support_pkg 是 AimRT 中用于识别数据类型的工具。在 AimRT 的 reocrd_playback ，echo 和 proxy 插件中都有使用 type_support_pkg 来识别数据类型，目前对于每一个用于 channel 的消息类型，AimRT会以目录为单位生成相应的 type_support_pkg，以供上述插件使用，其中 src/protocols/ros2/ros2_msgs 目录下是 ros2 humble 自带的一些消息类型。