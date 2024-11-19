#!/bin/zsh

source /opt/ros/humble/setup.zsh
source install/share/example_ros2/local_setup.zsh

./aimrt_main --cfg_file_path=./cfg/examples_plugins_proxy_plugin_cfg.yaml
