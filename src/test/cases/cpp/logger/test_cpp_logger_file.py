#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pytest
from pathlib import Path
from test_helpers.fixtures.aimrt_test import AimRTTestRunner
from test_helpers.core.callback_manager import CallbackTrigger, CallbackResult
from typing import Dict, Any
import os
import time


CASES = [
    'examples_cpp_logger_rotate_file.yaml',
    'examples_cpp_logger_rotate_file_with_sync.yaml',
]


def file_check(ctx: Dict[str, Any]) -> CallbackResult:
    p = ctx.get('process_info')

    if not p:
        return CallbackResult(False, 'missing process_info')

    cwd_env = (os.environ.get('CWD') or '').strip()
    base = Path(cwd_env or os.getcwd()).resolve()

    log_file_path = Path(base) / 'log' / 'examples_cpp_logger_rotate_file.log'

    ok = log_file_path.exists()
    if not ok:
        return CallbackResult(False, 'no recent non-empty log file found ', data={
            'script_path': getattr(p, 'script_path', ''),
            'log_file_path': log_file_path,
        })

    with open(log_file_path, 'r') as f:
        content = f.read()
        ok = 'Test fatal log' in content

    msg = 'log file verification passed' if ok else 'no Test fatal log in log file'

    log_file_path.unlink()

    return CallbackResult(ok, msg, data={
        'script_path': getattr(p, 'script_path', ''),
        'log_file_path': log_file_path,
    })


@pytest.mark.parametrize('yaml_name', CASES)
def test_logger_file_examples(yaml_name: str, aimrt_test_runner: AimRTTestRunner):
    yaml_path = (Path(__file__).parent / yaml_name).resolve()
    if not yaml_path.exists():
        pytest.skip(f'YAML not found: {yaml_path}')
    if not aimrt_test_runner.setup_from_yaml(str(yaml_path)):
        pytest.fail('Failed to setup test environment from YAML configuration')

    aimrt_test_runner.register_function_callback('test_cpp_logger_file', CallbackTrigger.PROCESS_END, file_check)

    success = aimrt_test_runner.run_test()
    if not success:
        pytest.fail('Test execution failed')
