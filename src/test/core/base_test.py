#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Base test class for the AimRT test framework

Provides YAML-driven test execution with integrated resource monitoring and process management.
"""

import time
from typing import Dict, List, Optional, Any

from .config_manager import ConfigManager, TestConfig
from .resource_monitor import ResourceMonitor
from .process_manager import ProcessManager, ProcessInfo
from .callback_manager import CallbackManager, CallbackTrigger
from .report_generator import ReportGenerator
from .pytest_results import export_results as export_pytest_results


class BaseAimRTTest:
    """Base class for AimRT tests"""

    def __init__(self):
        """Initialize test base class"""
        self.config_manager = ConfigManager()
        self.resource_monitor = ResourceMonitor(sample_interval=1.0)
        self.callback_manager = CallbackManager()
        self.process_manager = None
        self.report_generator = ReportGenerator()
        self._test_config: Optional[TestConfig] = None
        self._execution_results: Dict[str, ProcessInfo] = {}
        self._reports: List[Dict[str, Any]] = []

    def setup_from_yaml(self, yaml_path: str) -> bool:
        """
        Set up test environment from a YAML configuration file

        Args:
            yaml_path: path to the YAML configuration file

        Returns:
            bool: True on success
        """
        try:
            # Load configuration
            if not self.config_manager.load_config(yaml_path):
                print("❌ Failed to load YAML configuration")
                return False

            # Validate configuration
            if not self.config_manager.validate_config():
                print("❌ YAML configuration validation failed")
                return False

            # Get configuration
            self._test_config = self.config_manager.get_config()
            if not self._test_config:
                print("❌ Failed to obtain test configuration")
                return False

            # In pytest,默认不回显子进程日志到控制台，避免刷屏；需要时可在后续扩展为从 YAML 或 env 控制
            self.process_manager = ProcessManager(
                base_cwd=self._test_config.cwd,
                resource_monitor=self.resource_monitor,
                callback_manager=self.callback_manager,
                global_shutdown_patterns=getattr(self._test_config, 'global_shutdown_patterns', []) or [],
                stop_all_on_shutdown=bool(getattr(self._test_config, 'stop_all_on_shutdown', False)),
                echo_child_output=False
            )

            print(f"✅ Test environment setup succeeded: {self._test_config.name}")
            return True

        except Exception as e:
            print(f"❌ Failed to set up test environment: {e}")
            return False

    def run_test(self) -> bool:
        """
        Run the test

        Returns:
            bool: True if all executions succeeded
        """
        if not self._test_config or not self.process_manager:
            print("❌ Test environment not initialized")
            return False

        print(f"\n🎯 Start executing test: {self._test_config.name}")
        print(f"📝 Description: {self._test_config.description}")
        print(f"🔢 Execution count: {self._test_config.execution_count}")
        print(f"⏱️ Run time: {self._test_config.time_sec}s")

        success_count = 0

        try:
            for execution in range(self._test_config.execution_count):
                if self._test_config.execution_count > 1:
                    print(f"\n📊 Executing run {execution + 1}/{self._test_config.execution_count}")

                try:
                    self.callback_manager.clear_callback_results()
                except Exception:
                    pass

                # Execute script group
                results = self.process_manager.execute_scripts_with_dependencies(
                    self._test_config.scripts
                )

                self._execution_results.update(results)

                execution_success = True
                for script_path, process_info in results.items():
                    if process_info.status not in ["completed"]:
                        print(f"❌ Script execution failed: {script_path} (status: {process_info.status})")
                        execution_success = False
                    else:
                        print(f"✅ Script execution succeeded: {script_path}")

                # 统计回调失败（默认失败）：若任一回调结果 success=False，则判定本轮失败
                try:
                    cb_results = self.process_manager.get_callback_results() if self.process_manager else {}
                    registered = {}
                    try:
                        registered = self.callback_manager.get_registered_callbacks() or {}
                    except Exception:
                        registered = {}
                    has_callback_failure = False
                    for name, items in (cb_results or {}).items():
                        # soft_fail=True 的回调不影响整体成功
                        soft = False
                        try:
                            cfg = registered.get(name)
                            if cfg and getattr(cfg, 'params', None):
                                soft = bool(cfg.params.get('soft_fail', False))
                        except Exception:
                            soft = False
                        if soft:
                            continue
                        for r in (items or []):
                            try:
                                if r is not None and (not getattr(r, 'success', False)):
                                    has_callback_failure = True
                                    break
                            except Exception:
                                # 防御性：异常也视为失败
                                has_callback_failure = True
                                break
                        if has_callback_failure:
                            break
                    if has_callback_failure:
                        print("❌ Callback failures detected, mark this run as failed")
                        execution_success = False
                except Exception as e:
                    # 回调统计异常，保守处理为失败
                    print(f"❌ Callback evaluation error: {e}")
                    execution_success = False

                if execution_success:
                    success_count += 1
                    print(f"✅ Run {execution + 1} succeeded")
                else:
                    print(f"❌ Run {execution + 1} failed")

                if execution < self._test_config.execution_count - 1:
                    print("⏳ Waiting 2 seconds before the next run...")
                    time.sleep(2)

            self._generate_reports()

            success_rate = (success_count / self._test_config.execution_count) * 100
            print("\n📈 Test completion statistics:")
            print(f"   Total runs: {self._test_config.execution_count}")
            print(f"   Success runs: {success_count}")
            print(f"   Success rate: {success_rate:.1f}%")

            return success_count == self._test_config.execution_count

        except Exception as e:
            print(f"❌ Error while executing test: {e}")
            return False

    def get_process_status(self) -> Dict[str, str]:
        """
        Get process statuses

        Returns:
            Dict[str, str]: mapping from script path to status
        """
        if not self.process_manager:
            return {}

        processes = self.process_manager.get_all_processes()
        return {path: info.status for path, info in processes.items()}

    def get_execution_results(self) -> Dict[str, ProcessInfo]:
        """Get execution results"""
        return self._execution_results.copy()

    def get_reports(self) -> List[Dict[str, Any]]:
        """Get all generated reports"""
        return self._reports

    def register_callback(self, callback):
        """Register a custom callback"""
        if self.process_manager:
            self.process_manager.register_callback(callback)
        else:
            self.callback_manager.register_callback(callback)

    def register_function_callback(self, name: str, trigger: CallbackTrigger, func, **kwargs):
        """Register a function callback (supports per-script enabled_callbacks allowlist)"""
        if self._test_config:
            # Determine allowlist mode: enabled if any script in YAML has an enabled_callbacks field (even empty)
            whitelist_enabled = any(hasattr(sc, 'enabled_callbacks') and sc.enabled_callbacks is not None
                                    for sc in (self._test_config.scripts or []))

            if whitelist_enabled:
                # Collect each script's enabled_callbacks (empty means no callbacks allowed for that script)
                script_enabled_map = {}
                for sc in (self._test_config.scripts or []):
                    if sc.enabled_callbacks is None:
                        continue
                    for n in (sc.enabled_callbacks or []):
                        script_enabled_map.setdefault(n, set()).add(sc.path)

                # If allowlist is enabled but this callback is not listed by any script, do not register
                if name not in script_enabled_map and name not in kwargs.get('enabled_callbacks', []):
                    print(
                        f"⏭️ Skip registering callback: {name} (allowlist enabled, not listed in any script's enabled_callbacks)")
                    return None
                # Inject target script list for filtering during execution
                kwargs = dict(kwargs or {})
                params = dict(kwargs.get('params', {}))
                params['target_scripts'] = sorted(list(script_enabled_map[name]))
                kwargs['params'] = params
        if self.process_manager:
            return self.process_manager.register_function_callback(name, trigger, func, **kwargs)
        else:
            return self.callback_manager.register_function_callback(name, trigger, func, **kwargs)

    def get_callback_results(self, callback_name: Optional[str] = None):
        """Get callback execution results"""
        if self.process_manager:
            return self.process_manager.get_callback_results(callback_name)
        else:
            return self.callback_manager.get_callback_results(callback_name).copy()

    def _generate_reports(self):
        """Generate test reports"""
        if not self.process_manager:
            return

        print("\n📊 Generating reports...")

        # generate summary report
        summary_report = self.process_manager.generate_summary_report()
        # Inject meta info: source YAML path and test name
        try:
            if self._test_config:
                meta = summary_report.setdefault("meta", {})
                meta["test_name"] = self._test_config.name
                src = getattr(self._test_config, "source_yaml_path", "") or ""
                meta["source_yaml_path"] = src
        except Exception:
            pass
        self._reports.append({
            "type": "summary",
            "timestamp": time.time(),
            "data": summary_report
        })

        self._print_resource_usage_report(summary_report)

        if self._test_config:
            callback_results = self.process_manager.get_callback_results() if self.process_manager else {}
            # 将回调失败统计注入 meta，供 index 聚合页使用
            try:
                cb_fail_cnt = 0
                for _name, items in (callback_results or {}).items():
                    for r in (items or []):
                        try:
                            if r is not None and (not getattr(r, 'success', False)):
                                cb_fail_cnt += 1
                        except Exception:
                            cb_fail_cnt += 1
                if cb_fail_cnt > 0:
                    m = summary_report.setdefault("meta", {})
                    m["callback_failures"] = True
                    m["callback_failed_count"] = cb_fail_cnt
            except Exception:
                pass
            # Export pytest results
            try:
                pytest_results = export_pytest_results()
            except Exception:
                pytest_results = {"summary": {}, "tests": []}

            report_files = self.report_generator.generate_all_reports(
                self._test_config.name,
                self._execution_results,
                summary_report,
                callback_results,
                pytest_results
            )
            for format_type, filepath in report_files.items():
                print(f"📋 {format_type.upper()} report: {filepath}")

        print("✅ Reports generated")

    def _print_resource_usage_report(self, summary_report: Dict[str, Any]):
        """Print resource usage report"""
        print("\n📋 Resource usage report:")
        print("=" * 80)

        summary = summary_report["summary"]
        print(f"Total processes: {summary['total_processes']}")
        print(f"Completed: {summary['completed']}")
        print(f"Failed: {summary['failed']}")
        print(f"Timeout: {summary['timeout']}")
        print(f"Killed: {summary['killed']}")
        print(f"Success rate: {summary['success_rate']:.1f}%")
        print(f"Total duration: {summary['total_duration_seconds']:.2f}s")

        print("\n" + "─" * 80)
        print("Per-process resource usage:")
        print("─" * 80)

        # Print resource usage per process
        for script_path, process_data in summary_report["processes"].items():
            print(f"\n🔹 Script: {script_path}")
            print(f"   Status: {process_data['status']}")
            print(f"   PID: {process_data['pid']}")
            print(
                f"   Duration: {process_data['duration_seconds']:.2f}s" if process_data['duration_seconds'] else "   Duration: N/A")

            if "resource_usage" in process_data:
                resource = process_data["resource_usage"]

                # CPU usage
                cpu = resource.get("cpu", {})
                print(f"   💻 CPU: avg {cpu.get('avg_percent', 0):.1f}%, "
                      f"max {cpu.get('max_percent', 0):.1f}%, "
                      f"final {cpu.get('final_percent', 0):.1f}%")

                # Memory usage
                memory = resource.get("memory", {})
                print(f"   🧠 Memory: avg {memory.get('avg_rss_mb', 0):.1f}MB, "
                      f"max {memory.get('max_rss_mb', 0):.1f}MB, "
                      f"final {memory.get('final_rss_mb', 0):.1f}MB")
                print(f"   📊 Memory percent: avg {memory.get('avg_percent', 0):.1f}%, "
                      f"max {memory.get('max_percent', 0):.1f}%, "
                      f"final {memory.get('final_percent', 0):.1f}%")

                # Disk I/O
                disk = resource.get("disk", {})
                print(f"   💾 Disk read: total {disk.get('total_read_mb', 0):.2f}MB, "
                      f"avg {disk.get('avg_read_mb_per_sec', 0):.2f}MB/s")
                print(f"   💾 Disk write: total {disk.get('total_write_mb', 0):.2f}MB, "
                      f"avg {disk.get('avg_write_mb_per_sec', 0):.2f}MB/s")
                print(f"   🔢 I/O count: read {disk.get('total_read_count', 0)}, "
                      f"write {disk.get('total_write_count', 0)}")
            else:
                print("   ⚠️ No resource monitoring data")

        print("\n" + "=" * 80)

    def cleanup(self):
        """Clean up the test environment"""
        print("\n🧹 Cleaning up test environment...")

        if self.process_manager:
            self.process_manager.cleanup()

        # Stop all resource monitoring
        self.resource_monitor.stop_all_monitoring()

        print("✅ Test environment cleanup completed")

    def __enter__(self):
        """Context manager enter"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.cleanup()

    # pytest fixture hooks
    def setup_method(self, method):
        """pytest method-level setup"""
        pass

    def teardown_method(self, method):
        """pytest method-level teardown"""
        self.cleanup()
        time.sleep(1)

    def setup_class(self):
        """pytest class-level setup"""
        pass

    def teardown_class(self):
        """pytest class-level teardown"""
        self.cleanup()
