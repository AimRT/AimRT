#!/bin/bash

export AIMRT_PLUGIN_DIR=$(pip show aimrt_py | grep Location | awk '{print $2}')/aimrt_py

python3 benchmark_publisher_app.py --cfg_file_path ./cfg/benchmark_publisher_cfg.yaml
