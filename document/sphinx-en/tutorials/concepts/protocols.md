# Protocols in AimRT

Protocols in AimRT primarily serve to enable data exchange and communication between components. Whether communicating within the same process or across processes, AimRT provides a unified interface and data serialization/deserialization mechanism to ensure data is correctly transmitted and parsed. AimRT currently supports two protocols: ros2 and protobuf.

Refer to the `src/protocols` directory in the AimRT source code.


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


## type_support Concept
type_support is an important component in AimRT used for serializing and deserializing messages. In AimRT's record_playback, echo, and proxy plugins, type_support_pkg is used to identify data types. Currently, for each message type used in channels, AimRT generates a corresponding type_support_pkg on a per-directory basis for use by the aforementioned plugins. The generated pkg is named `lib{directory_name}_{protocol_type}_ts.so`, where the src/protocols/ros2/ros2_msgs directory contains some message types that come with ros2 humble.