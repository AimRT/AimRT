#!/bin/bash

export AIMRT_PLUGIN_DIR=$(pip show aimrt_py | grep Location | awk '{print $2}')/aimrt_py
source $AIMRT_PLUGIN_DIR/share/example_ros2/local_setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON:-python3}"

"${PYTHON_BIN}" "${SCRIPT_DIR}/examples_py_ros2_rpc_server_app.py" \
  --cfg_file_path="${SCRIPT_DIR}/cfg/examples_py_ros2_rpc_grpc_server_cfg.yaml"
