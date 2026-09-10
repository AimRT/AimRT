#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pytest
from pathlib import Path
from test_helpers.fixtures.aimrt_test import AimRTTestRunner
from test_helpers.core.callback_manager import CallbackTrigger, CallbackResult
from typing import Dict, Any
import gzip
import os
import time


CASES = [
    'examples_cpp_logger_rotate_file.yaml',
    'examples_cpp_logger_rotate_file_with_sync.yaml',
]

COMPRESSION_CASE = 'examples_cpp_logger_rotate_file_with_compression.yaml'
COMPRESSION_LOG_NAME = 'examples_cpp_logger_rotate_file_with_compression.log'


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


def compression_check(ctx: Dict[str, Any]) -> CallbackResult:
    p = ctx.get('process_info')

    if not p:
        return CallbackResult(False, 'missing process_info')

    cwd_env = (os.environ.get('CWD') or '').strip()
    base = Path(cwd_env or os.getcwd()).resolve()

    log_dir = Path(base) / 'log'
    rotated_files = list(log_dir.glob(COMPRESSION_LOG_NAME + '_*'))
    compressed_files = [f for f in rotated_files if f.suffix == '.gz']
    uncompressed_files = [f for f in rotated_files if f.suffix != '.gz']

    data = {
        'script_path': getattr(p, 'script_path', ''),
        'compressed_files': [f.name for f in compressed_files],
        'uncompressed_files': [f.name for f in uncompressed_files],
    }

    if not compressed_files:
        return CallbackResult(False, 'no compressed log file found', data=data)

    # the rotated log file should be removed once it is compressed
    if uncompressed_files:
        return CallbackResult(False, 'rotated log file is not removed after compression', data=data)

    ok = True
    msg = 'compressed log file verification passed'
    for compressed_file in compressed_files:
        try:
            with gzip.open(compressed_file, 'rb') as f:
                while f.read(1024 * 1024):
                    pass
        except OSError as e:
            ok = False
            msg = f'broken compressed log file {compressed_file.name}: {e}'
            break

    for f in rotated_files:
        f.unlink()
    (log_dir / COMPRESSION_LOG_NAME).unlink(missing_ok=True)

    return CallbackResult(ok, msg, data=data)


def test_logger_file_compression_example(aimrt_test_runner: AimRTTestRunner):
    yaml_path = (Path(__file__).parent / COMPRESSION_CASE).resolve()
    if not yaml_path.exists():
        pytest.skip(f'YAML not found: {yaml_path}')
    if not aimrt_test_runner.setup_from_yaml(str(yaml_path)):
        pytest.fail('Failed to setup test environment from YAML configuration')

    aimrt_test_runner.register_function_callback(
        'test_cpp_logger_file_compression',
        CallbackTrigger.PROCESS_END,
        compression_check)

    success = aimrt_test_runner.run_test()
    if not success:
        pytest.fail('Test execution failed')
