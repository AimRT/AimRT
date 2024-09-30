#!/bin/bash

set -e

source install/share/example_ros2/local_setup.bash
source install/share/ros2_plugin_proto/local_setup.bash

./aimrt_main --cfg_file_path=./cfg/examples_plugins_ros2_plugin_ros2_chn_sub_cfg.yaml
