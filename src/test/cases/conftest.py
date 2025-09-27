#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Ensure the parent `src/test` directory is on sys.path when running pytest from `src/test/cases`.
This allows imports like `from test_helpers.fixtures.aimrt_test import AimRTTestRunner` to work.
"""

from test_helpers.fixtures.aimrt_test import aimrt_test_runner, aimrt_test_runner_session
import sys
from pathlib import Path

PARENT_TEST_DIR = Path(__file__).resolve().parent.parent
if str(PARENT_TEST_DIR) not in sys.path:
    sys.path.insert(0, str(PARENT_TEST_DIR))

# Re-export fixtures so tests under this subtree can use them

__all__ = [
    'aimrt_test_runner',
    'aimrt_test_runner_session',
]
