#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pytest
from pathlib import Path
from test_helpers.fixtures.aimrt_test import AimRTTestRunner

CASES = [
    'examples_cpp_context_executor.yaml',
    'examples_cpp_context_logger.yaml',
    'examples_cpp_context_publisher.yaml',
    'examples_cpp_context_subscriber_on_executor.yaml',
    'examples_cpp_context_subscriber_inline.yaml',
    'examples_cpp_context_rpc_server_on_executor.yaml',
    'examples_cpp_context_rpc_server_inline.yaml',
    'examples_cpp_context_rpc_client.yaml',
]


@pytest.mark.parametrize('yaml_name', CASES)
def test_executor_examples(yaml_name: str, aimrt_test_runner: AimRTTestRunner):
    yaml_path = (Path(__file__).parent / yaml_name).resolve()
    if not yaml_path.exists():
        pytest.skip(f'YAML not found: {yaml_path}')
    if not aimrt_test_runner.setup_from_yaml(str(yaml_path)):
        pytest.fail('Failed to setup test environment from YAML configuration')
    success = aimrt_test_runner.run_test()
    if not success:
        pytest.fail('Test execution failed')
