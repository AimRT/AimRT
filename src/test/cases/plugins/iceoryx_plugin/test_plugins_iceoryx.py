#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pytest
from pathlib import Path
from test_helpers.fixtures.aimrt_test import AimRTTestRunner

CASES = [
    'examples_plugins_iceoryx_plugin_pb_chn.yaml',
    'examples_plugins_iceoryx_plugin_ros2_chn.yaml',
    'examples_plugins_iceoryx_plugin_pb_chn_benchmark.yaml',
]


@pytest.mark.parametrize('yaml_name', CASES)
def test_iceoryx_plugin_examples(yaml_name: str, aimrt_test_runner: AimRTTestRunner):
    yaml_path = (Path(__file__).parent / yaml_name).resolve()
    if not yaml_path.exists():
        pytest.skip(f'YAML not found: {yaml_path}')
    if not aimrt_test_runner.setup_from_yaml(str(yaml_path)):
        pytest.fail('Failed to setup test environment from YAML configuration')
    success = aimrt_test_runner.run_test()
    if not success:
        pytest.fail('Test execution failed')
