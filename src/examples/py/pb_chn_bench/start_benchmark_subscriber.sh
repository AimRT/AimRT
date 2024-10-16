#!/bin/bash

export AIMRT_PLUGIN_DIR=$(pip show aimrt_py | grep Location | awk '{print $2}')/aimrt_py

python3 benchmark_subscriber_app.py --cfg_file_path ./cfg/benchmark_subscriber_cfg.yaml
