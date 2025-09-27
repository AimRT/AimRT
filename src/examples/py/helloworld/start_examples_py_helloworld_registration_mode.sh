#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON:-python3}"

"${PYTHON_BIN}" "${SCRIPT_DIR}/examples_py_helloworld_registration_mode.py" \
  --cfg_file_path="${SCRIPT_DIR}/cfg/examples_py_helloworld_registration_mode_cfg.yaml"
