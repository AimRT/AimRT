#!/bin/bash

export AIMRT_PLUGIN_DIR=$(pip show aimrt_py | grep Location | awk '{print $2}')/aimrt_py

export AMENT_CURRENT_PREFIX=${AIMRT_PLUGIN_DIR}
source ${AIMRT_PLUGIN_DIR}/share/ros2_plugin_proto/local_setup.bash

python3 ./examples_py_protobuf_rpc_server_app.py --cfg_file_path=./cfg/examples_py_protobuf_rpc_ros2_server_cfg.yaml
