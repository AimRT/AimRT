#!/bin/bash

set -e

source install/share/ros2_plugin_proto/local_setup.bash

./aimrt_main --cfg_file_path=./cfg/examples_plugins_ros2_plugin_protobuf_channel_benchmark_sub_cfg.yaml
