#!/bin/bash

export AIMRT_PLUGIN_DIR=$(pip show aimrt_py | grep Location | awk '{print $2}')/aimrt_py

source ${AIMRT_PLUGIN_DIR}/share/ros2_plugin_proto/local_setup.bash

python3 ./examples_py_protobuf_channel_publisher_app.py --cfg_file_path=./cfg/examples_py_protobuf_channel_ros2_pub_cfg.yaml
