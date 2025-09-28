#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum


class CallbackTrigger(Enum):
    """Callback trigger timing"""
    PROCESS_START = "process_start"      # when a process starts
    PROCESS_END = "process_end"          # when a process ends
    PERIODIC = "periodic"                # periodic check


@dataclass
class CallbackResult:
    """Callback execution result"""
    success: bool
    message: str
    data: Dict[str, Any] = field(default_factory=dict)
    timestamp: datetime = field(default_factory=datetime.now)
    warnings: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)


@dataclass
class CallbackConfig:
    """Callback configuration"""
    name: str
    trigger: CallbackTrigger
    interval_sec: float = 1.0  # interval seconds for periodic checks
    timeout_sec: float = 10.0  # callback execution timeout seconds
    retry_count: int = 0
    params: Dict[str, Any] = field(default_factory=dict)


class BaseCallback(ABC):
    """Base callback type"""

    def __init__(self, config: CallbackConfig):
        self.config = config
        self.results: List[CallbackResult] = []
        self._lock = threading.Lock()

    @abstractmethod
    def execute(self, context: Dict[str, Any]) -> CallbackResult:
        """
        Execute callback logic

        Args:
            context: execution context, includes process info, monitor data, etc.

        Returns:
            CallbackResult: execution result
        """
        pass

    def get_results(self) -> List[CallbackResult]:
        """Get all results"""
        with self._lock:
            return self.results.copy()

    def add_result(self, result: CallbackResult):
        """Append a result"""
        with self._lock:
            self.results.append(result)

    def clear_results(self):
        """Clear results"""
        with self._lock:
            self.results.clear()


class CustomFunctionCallback(BaseCallback):
    """Custom function-based callback"""

    def __init__(self, config: CallbackConfig, func: Callable[[Dict[str, Any]], CallbackResult]):
        super().__init__(config)
        self.func = func

    def execute(self, context: Dict[str, Any]) -> CallbackResult:
        """Execute the custom function"""
        try:
            return self.func(context)
        except Exception as e:
            return CallbackResult(
                success=False,
                message=f"Exception while executing custom function: {e}",
                errors=[str(e)]
            )


class CallbackManager:
    """Callback manager"""

    def __init__(self):
        self._callbacks: Dict[str, BaseCallback] = {}
        self._periodic_threads: Dict[str, threading.Thread] = {}
        self._stop_events: Dict[str, threading.Event] = {}
        self._lock = threading.Lock()

    def register_callback(self, callback: BaseCallback):
        """Register a callback"""
        with self._lock:
            self._callbacks[callback.config.name] = callback
            print(f"✅ Registered callback: {callback.config.name} ({callback.config.trigger.value})")

    def register_function_callback(self, name: str, trigger: CallbackTrigger,
                                   func: Callable[[Dict[str, Any]], CallbackResult],
                                   **kwargs) -> BaseCallback:
        """Register a function callback"""
        config = CallbackConfig(name=name, trigger=trigger, **kwargs)
        callback = CustomFunctionCallback(config, func)
        self.register_callback(callback)
        return callback

    def unregister_callback(self, name: str):
        """Unregister a callback"""
        with self._lock:
            if name in self._callbacks:
                if name in self._periodic_threads:
                    self._stop_events[name].set()
                    self._periodic_threads[name].join(timeout=1)
                    del self._periodic_threads[name]
                    del self._stop_events[name]

                del self._callbacks[name]
                print(f"🗑️ Unregistered callback: {name}")

    def start_periodic_callbacks(self, context_provider: Callable[[], Dict[str, Any]]):
        """Start periodic callbacks"""
        with self._lock:
            for name, callback in self._callbacks.items():
                if (callback.config.trigger == CallbackTrigger.PERIODIC and
                        name not in self._periodic_threads):

                    stop_event = threading.Event()
                    self._stop_events[name] = stop_event

                    thread = threading.Thread(
                        target=self._periodic_callback_worker,
                        args=(callback, context_provider, stop_event),
                        daemon=True,
                        name=f"callback-{name}"
                    )
                    self._periodic_threads[name] = thread
                    thread.start()
                    print(f"🔄 Started periodic callback: {name}")

    def stop_periodic_callbacks(self):
        """Stop all periodic callbacks"""
        with self._lock:
            for name, stop_event in self._stop_events.items():
                stop_event.set()

            for name, thread in self._periodic_threads.items():
                thread.join(timeout=1)
                print(f"⏹️ Stopped periodic callback: {name}")

            self._periodic_threads.clear()
            self._stop_events.clear()

    def execute_callbacks(self, trigger: CallbackTrigger, context: Dict[str, Any]) -> Dict[str, CallbackResult]:
        """Execute callbacks for the specified trigger"""
        results = {}

        with self._lock:
            callbacks_to_execute = [
                (name, callback) for name, callback in self._callbacks.items()
                if callback.config.trigger == trigger
            ]

        for name, callback in callbacks_to_execute:
            try:
                print(f"🔍 Executing callback: {name}")
                enriched_context = dict(context)
                pinfo = context.get('process_info')
                if pinfo:
                    enriched_context.setdefault('script_path', getattr(pinfo, 'script_path', None))
                    enriched_context.setdefault('pid', getattr(pinfo, 'pid', None))
                    params = getattr(callback.config, 'params', {}) or {}
                    try:
                        value = params.get('target_scripts', params.get('target_script', []))
                        if isinstance(value, (list, tuple, set)):
                            target_scripts = [str(x) for x in value]
                        elif value:
                            target_scripts = [str(value)]
                        else:
                            target_scripts = []
                    except Exception:
                        target_scripts = []
                    if target_scripts:
                        current_script = getattr(pinfo, 'script_path', None)
                        if current_script not in set(target_scripts):
                            continue
                result = callback.execute(enriched_context)
                if result is None:
                    result = CallbackResult(
                        success=False,
                        message="回调未返回结果",
                        errors=["callback returned None"]
                    )
                if pinfo:
                    try:
                        result.data.setdefault('script_path', getattr(pinfo, 'script_path', None))
                    except Exception:
                        result.data = {'script_path': getattr(pinfo, 'script_path', None)}
                callback.add_result(result)
                results[name] = result

                if result.success:
                    print(f"✅ Callback succeeded: {name} - {result.message}")
                else:
                    soft_fail = False
                    try:
                        soft_fail = callback.config.params.get('soft_fail', False)
                    except Exception:
                        soft_fail = False
                    if soft_fail:
                        print(f"⚠️ Callback soft-failed: {name} - {result.message}")
                    else:
                        print(f"❌ Callback failed: {name} - {result.message}")

                if result.warnings:
                    for warning in result.warnings:
                        print(f"⚠️ Warning: {warning}")

            except BaseException as e:
                error_result = CallbackResult(
                    success=False,
                    message=f"回调执行异常: {e}",
                    errors=[str(e)]
                )
                callback.add_result(error_result)
                results[name] = error_result
                print(f"❌ Callback exception: {name} - {e}")

        return results

    def get_callback_results(self, callback_name: Optional[str] = None) -> Dict[str, List[CallbackResult]]:
        """Get callback results"""
        results = {}

        with self._lock:
            if callback_name:
                if callback_name in self._callbacks:
                    results[callback_name] = self._callbacks[callback_name].get_results()
            else:
                for name, callback in self._callbacks.items():
                    results[name] = callback.get_results()

        return results

    def clear_callback_results(self, callback_name: Optional[str] = None):
        """Clear callback results"""
        with self._lock:
            if callback_name:
                if callback_name in self._callbacks:
                    self._callbacks[callback_name].clear_results()
            else:
                for callback in self._callbacks.values():
                    callback.clear_results()

    def _periodic_callback_worker(self, callback: BaseCallback,
                                  context_provider: Callable[[], Dict[str, Any]],
                                  stop_event: threading.Event):
        """Worker for periodic callbacks"""
        while not stop_event.wait(callback.config.interval_sec):
            try:
                base_context = context_provider()

                # Read target script allowlist (aligned with execute_callbacks)
                params = getattr(callback.config, 'params', {}) or {}
                try:
                    value = params.get('target_scripts', params.get('target_script', []))
                    if isinstance(value, (list, tuple, set)):
                        target_scripts = [str(x) for x in value]
                    elif value:
                        target_scripts = [str(value)]
                    else:
                        target_scripts = []
                except Exception:
                    target_scripts = []
                target_set = set(target_scripts)

                # Execute periodic callbacks per process, filtered by target_scripts
                processes = (
                    base_context.get('active_processes')
                    or base_context.get('all_processes')
                    or []
                )

                # If processes exist, execute per process; otherwise execute once only when no target_scripts limit
                if processes:
                    for pinfo in processes:
                        try:
                            if target_set and getattr(pinfo, 'script_path', None) not in target_set:
                                continue

                            enriched_context = dict(base_context)
                            enriched_context['process_info'] = pinfo
                            enriched_context.setdefault('script_path', getattr(pinfo, 'script_path', None))
                            enriched_context.setdefault('pid', getattr(pinfo, 'pid', None))

                            result = callback.execute(enriched_context)
                            callback.add_result(result)
                            if not result.success:
                                print(f"⚠️ Periodic callback failed: {callback.config.name} - {result.message}")
                        except BaseException as e:
                            error_result = CallbackResult(
                                success=False,
                                message=f"周期性回调异常: {e}",
                                errors=[str(e)]
                            )
                            callback.add_result(error_result)
                            print(f"❌ Periodic callback exception: {callback.config.name} - {e}")
                else:
                    # No processes: execute once only when no target_scripts limit (backward compatible global checks)
                    if target_set:
                        continue
                    result = callback.execute(base_context)
                    callback.add_result(result)
                    if not result.success:
                        print(f"⚠️ Periodic callback failed: {callback.config.name} - {result.message}")

            except BaseException as e:
                error_result = CallbackResult(
                    success=False,
                    message=f"周期性回调异常: {e}",
                    errors=[str(e)]
                )
                callback.add_result(error_result)
                print(f"❌ Periodic callback exception: {callback.config.name} - {e}")

    def get_registered_callbacks(self) -> Dict[str, CallbackConfig]:
        """Get registered callback configurations"""
        with self._lock:
            return {name: callback.config for name, callback in self._callbacks.items()}
