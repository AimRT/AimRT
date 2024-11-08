#!/bin/bash

export AIMRT_PLUGIN_DIR=$(pip show aimrt_py | grep Location | awk '{print $2}')/aimrt_py

python3 ./examples_py_ros2_chn_subscriber_app.py --cfg_file_path=./cfg/examples_py_ros2_chn_http_sub_cfg.yaml
