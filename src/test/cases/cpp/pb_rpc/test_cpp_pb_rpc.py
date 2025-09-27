#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pytest
from pathlib import Path
from pytest_aimrt.fixtures.aimrt_test import AimRTTestRunner

CASES = [
    'examples_cpp_pb_rpc_benchmark_single_pkg.yaml',
    'examples_cpp_pb_rpc_benchmark.yaml',
    'examples_cpp_pb_rpc_sync.yaml',
    'examples_cpp_pb_rpc_async.yaml',
    'examples_cpp_pb_rpc_future.yaml',
    'examples_cpp_pb_rpc_co.yaml',
    'examples_cpp_pb_rpc_single_pkg.yaml',
    'examples_cpp_pb_proxy_rpc_co.yaml',
    'examples_cpp_pb_rpc_client_app.yaml',
    'examples_cpp_pb_rpc_server_app.yaml',
]

@pytest.mark.parametrize('yaml_name', CASES)
def test_pb_rpc_examples(yaml_name: str, aimrt_test_runner: AimRTTestRunner):
    yaml_path = (Path(__file__).parent / yaml_name).resolve()
    if not yaml_path.exists():
        pytest.skip(f'YAML not found: {yaml_path}')
    if not aimrt_test_runner.setup_from_yaml(str(yaml_path)):
        pytest.fail('Failed to setup test environment from YAML configuration')
    success = aimrt_test_runner.run_test()
    if not success:
        pytest.fail('Test execution failed')