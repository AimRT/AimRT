#!/bin/bash

export AIMRT_PLUGIN_DIR=$(pip show aimrt_py | grep Location | awk '{print $2}')/aimrt_py
source $AIMRT_PLUGIN_DIR/share/example_ros2/local_setup.bash

python3 ./examples_py_ros2_chn_publisher_app.py --cfg_file_path=./cfg/examples_py_ros2_chn_http_pub_cfg.yaml
