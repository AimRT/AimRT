#!/bin/bash

export AIMRT_PLUGIN_DIR=$(pip show aimrt_py | grep Location | awk '{print $2}')/aimrt_py


export AMENT_CURRENT_PREFIX=${AIMRT_PLUGIN_DIR}
source ${AIMRT_PLUGIN_DIR}/share/ros2_plugin_proto/local_setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON:-python3}"

"${PYTHON_BIN}" "${SCRIPT_DIR}/examples_py_pb_rpc_client_app.py" \
  --cfg_file_path="${SCRIPT_DIR}/cfg/examples_py_pb_rpc_ros2_client_cfg.yaml"
