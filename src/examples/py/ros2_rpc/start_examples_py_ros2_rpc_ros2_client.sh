#!/bin/bash

export AIMRT_PLUGIN_DIR=$(pip show aimrt_py | grep Location | awk '{print $2}')/aimrt_py

export AMENT_CURRENT_PREFIX=${AIMRT_PLUGIN_DIR}
source ${AIMRT_PLUGIN_DIR}/share/example_ros2/local_setup.bash
source ${AIMRT_PLUGIN_DIR}/share/ros2_plugin_proto/local_setup.bash

python3 ./examples_py_ros2_rpc_client_app.py --cfg_file_path=./cfg/examples_py_ros2_rpc_ros2_client_cfg.yaml
