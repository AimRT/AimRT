#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pytest
from pathlib import Path
from test_helpers.fixtures.aimrt_test import AimRTTestRunner

CASES = [
    'examples_py_pb_rpc_http.yaml',
    'examples_py_pb_rpc_grpc.yaml',
    'examples_py_pb_rpc_ros2.yaml',
]


@pytest.mark.parametrize('yaml_name', CASES)
def test_pb_chn_examples(yaml_name: str, aimrt_test_runner: AimRTTestRunner):
    yaml_path = (Path(__file__).parent / yaml_name).resolve()
    if not yaml_path.exists():
        pytest.skip(f'YAML not found: {yaml_path}')
    if not aimrt_test_runner.setup_from_yaml(str(yaml_path)):
        pytest.fail('Failed to setup test environment from YAML configuration')
    success = aimrt_test_runner.run_test()
    if not success:
        pytest.fail('Test execution failed')
