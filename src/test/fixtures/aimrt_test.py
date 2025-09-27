#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pytest fixtures for the AimRT test framework

Provide pytest-compatible fixtures, avoiding collection issues caused by inheriting base classes.
"""

import pytest
from typing import Dict

from ..core.base_test import BaseAimRTTest
from ..core.pytest_results import record_report, reset_results


class AimRTTestRunner:
    """AimRT test runner that wraps BaseAimRTTest for pytest usage"""

    def __init__(self):
        self._base_test = BaseAimRTTest()
        self._initialized = False

    def setup_from_yaml(self, yaml_path: str) -> bool:
        """Set up test environment from YAML"""
        result = self._base_test.setup_from_yaml(yaml_path)
        if result:
            self._initialized = True
        return result

    def run_test(self) -> bool:
        """Run the test"""
        if not self._initialized:
            raise RuntimeError("Test environment not initialized. Call setup_from_yaml first.")
        return self._base_test.run_test()

    def get_process_status(self) -> Dict[str, str]:
        """Get process status"""
        return self._base_test.get_process_status()

    def get_execution_results(self):
        """Get execution results"""
        return self._base_test.get_execution_results()

    def get_reports(self):
        """Get all reports"""
        return self._base_test.get_reports()

    def register_callback(self, callback):
        """Register a custom callback"""
        return self._base_test.register_callback(callback)

    def register_function_callback(self, name: str, trigger, func, **kwargs):
        """Register a function callback"""
        return self._base_test.register_function_callback(name, trigger, func, **kwargs)

    def get_callback_results(self, callback_name=None):
        """Get callback results"""
        return self._base_test.get_callback_results(callback_name)

    def cleanup(self):
        """Clean up the test environment"""
        self._base_test.cleanup()
        self._initialized = False


@pytest.fixture(scope="function")
def aimrt_test_runner():
    """
    AimRT test runner fixture

    Provides AimRT testing capability with automatic cleanup.

    Usage:
        def test_my_aimrt_feature(aimrt_test_runner):
            yaml_path = "path/to/test_config.yaml"
            assert aimrt_test_runner.setup_from_yaml(yaml_path)
            assert aimrt_test_runner.run_test()
    """
    runner = AimRTTestRunner()

    try:
        yield runner
    finally:
        # Auto cleanup
        runner.cleanup()


@pytest.fixture(scope="session")
def aimrt_test_runner_session():
    """
    Session-scoped AimRT test runner fixture

    Reuses the same instance across the test session, suitable for scenarios spanning multiple tests.
    Note: manual cleanup is required.
    """
    runner = AimRTTestRunner()

    try:
        yield runner
    finally:
        runner.cleanup()


def pytest_configure(config):
    """Pytest configure hook"""
    # Register custom markers
    config.addinivalue_line(
        "markers", "aimrt: mark test as AimRT framework test"
    )
    config.addinivalue_line(
        "markers", "slow: mark test as slow running"
    )
    config.addinivalue_line(
        "markers", "integration: mark test as integration test"
    )

    # Reset result collection once
    try:
        reset_results()
    except Exception:
        pass


def pytest_runtest_logreport(report):
    """Collect results for each test phase (aggregated for final report)."""
    try:
        # Record only the call phase; setup/teardown failures will override as error
        longrepr = None
        if hasattr(report, 'longrepr') and report.longrepr:
            try:
                longrepr = str(report.longrepr)
            except Exception:
                longrepr = None
        keywords = sorted([str(k) for k, v in getattr(report, 'keywords', {}).items() if v])
        record_report(
            nodeid=report.nodeid,
            outcome=str(report.outcome),
            duration=float(getattr(report, 'duration', 0.0) or 0.0),
            when=str(report.when),
            longrepr=longrepr,
            keywords=keywords,
        )
    except Exception:
        pass
