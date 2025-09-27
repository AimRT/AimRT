#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pytest configuration for the AimRT test framework

Provides global pytest configuration and fixtures.
"""

from test_helpers.fixtures.aimrt_test import aimrt_test_runner, aimrt_test_runner_session
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).parent))


__all__ = ['aimrt_test_runner', 'aimrt_test_runner_session']
