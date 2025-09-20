#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AimRT test framework

Pytest-based AimRT test framework supporting YAML configuration, resource monitoring, and process management.
"""

__version__ = "0.1.0"
__author__ = "AimRT"

from .core import (
    BaseAimRTTest,
    ConfigManager,
    TestConfig,
    ScriptConfig,
    ResourceMonitor,
    ProcessManager,
    CallbackManager,
    BaseCallback,
    CallbackResult,
    CallbackTrigger
)

from .fixtures.aimrt_test import AimRTTestRunner

__all__ = [
    'BaseAimRTTest',
    'ConfigManager',
    'TestConfig',
    'ScriptConfig',
    'ResourceMonitor',
    'ProcessManager',
    'CallbackManager',
    'BaseCallback',
    'CallbackResult',
    'CallbackTrigger',
    'AimRTTestRunner'
]