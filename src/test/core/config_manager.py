#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Configuration manager for the AimRT test framework

Loads and parses YAML test configuration files and provides configuration
information required by the test execution.
"""

import yaml
import os
import re
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from pathlib import Path

@dataclass
class HostProfile:
    """Remote host profile"""
    name: str = ""
    host: str = ""
    ssh_user: str = ""
    ssh_port: int = 22
    ssh_password: str = ""
    remote_cwd: str = ""

@dataclass
class ScriptConfig:
    """Script configuration"""
    path: str
    args: List[str] = field(default_factory=list)
    depends_on: List[str] = field(default_factory=list)
    delay_sec: int = 0
    time_sec: int = 60  # run time (seconds)
    kill_signal: int = 15  # termination signal (15=SIGTERM, 2=SIGINT)
    monitor: Dict[str, bool] = field(default_factory=lambda: {"cpu": True, "memory": True, "disk": True})
    environment: Dict[str, str] = field(default_factory=dict)
    cwd: str = ""
    shutdown_patterns: List[str] = field(default_factory=list)  # on match request graceful exit and mark completed
    # Whether to propagate shutdown as a "global shutdown" (terminate all scripts)
    propagate_shutdown: bool = False
    # Grace period after triggering shutdown (seconds) for cleanup
    shutdown_grace_sec: float = 0.0
    enabled_callbacks: Optional[List[str]] = None  # if present in YAML: list (possibly empty); None means no allowlist
    remote_env: Dict[str, str] = field(default_factory=dict)
    host_profile: Optional[HostProfile] = None


@dataclass
class TestConfig:
    """Test configuration"""
    name: str
    description: str = ""
    execution_count: int = 1
    time_sec: int = 60  # total run time (seconds)
    cwd: str = ""
    environment: Dict[str, str] = field(default_factory=dict)
    scripts: List[ScriptConfig] = field(default_factory=list)
    global_shutdown_patterns: List[str] = field(default_factory=list)
    stop_all_on_shutdown: bool = False
    # Absolute source YAML path, used by report index grouping
    source_yaml_path: str = ""


class ConfigManager:
    """Configuration manager"""

    def __init__(self):
        self._config: Optional[TestConfig] = None

    def load_config(self, config_path: str) -> bool:
        """
        Load a YAML configuration file

        Args:
            config_path: path to the configuration file

        Returns:
            bool: True on success, False otherwise
        """
        try:
            config_file = Path(config_path)
            if not config_file.exists():
                print(f"❌ Configuration file does not exist: {config_path}")
                return False

            with open(config_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)

            if not data:
                print("❌ Configuration file is empty")
                return False

            # Expand environment placeholders across the whole config tree
            data = self._expand_env_placeholders(data)

            self._config = self._parse_config(data)
            # Record the absolute path of the source YAML for report index grouping
            try:
                self._config.source_yaml_path = str(config_file.resolve())
            except Exception:
                self._config.source_yaml_path = str(config_file)
            print(f"✅ Configuration loaded: {self._config.name}")
            return True

        except yaml.YAMLError as e:
            print(f"❌ YAML parsing error: {e}")
            return False
        except Exception as e:
            print(f"❌ Failed to load configuration file: {e}")
            return False

    def _expand_env_placeholders(self, obj: Any) -> Any:
        """Recursively expand environment placeholders in strings, e.g., "${HOME}".

        Only replace keys that exist in the environment; otherwise keep the
        original placeholder text.
        """
        pattern = re.compile(r"\$\{([^}]+)\}")

        def replace_in_str(text: str) -> str:
            return pattern.sub(lambda m: os.environ.get(m.group(1), m.group(0)), text)

        if isinstance(obj, dict):
            return {k: self._expand_env_placeholders(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [self._expand_env_placeholders(v) for v in obj]
        if isinstance(obj, str):
            return replace_in_str(obj)
        return obj

    def _parse_config(self, data: Dict[str, Any]) -> TestConfig:
        """Parse configuration data"""
        config_data = data.get('config', {})

        test_config = TestConfig(
            name=data.get('name', 'Unknown Test'),
            description=data.get('description', ''),
            execution_count=config_data.get('execution_count', 1),
            time_sec=config_data.get('time_sec', 60),
            cwd=config_data.get('cwd', ''),
            environment=config_data.get('environment', {}),
            global_shutdown_patterns=config_data.get('global_shutdown_patterns', []) or [],
            stop_all_on_shutdown=bool(config_data.get('stop_all_on_shutdown', False))
        )

        input_data = data.get('input', {})
        scripts_data = input_data.get('scripts', [])

        hosts_profiles_raw = data.get('hosts', {}) or {}
        hosts_profiles: Dict[str, HostProfile] = {}
        for name, prof in hosts_profiles_raw.items():
            if not isinstance(prof, dict):
                continue
            # Port field robustness: may still be an unexpanded placeholder or empty
            raw_port = prof.get('ssh_port', 22)
            try:
                port_val = int(raw_port)
            except Exception:
                port_val = 22

            hosts_profiles[name] = HostProfile(
                name=name,
                host=prof.get('host', ''),
                ssh_user=prof.get('ssh_user', ''),
                ssh_port=port_val,
                ssh_password=prof.get('ssh_password', ''),
                remote_cwd=prof.get('remote_cwd', ''),
            )

        for script_data in scripts_data:
            script_remote = script_data.get('remote', '')
            host_prof = hosts_profiles.get(script_remote) if script_remote else None


            merged_remote_env: Dict[str, str] = {}
            prof_env = getattr(host_prof, 'remote_env', {}) if host_prof else {}
            if isinstance(prof_env, dict):
                merged_remote_env.update(prof_env)
            scr_env = script_data.get('remote_env', {}) or {}
            if isinstance(scr_env, dict):
                merged_remote_env.update(scr_env)

            if 'enabled_callbacks' in script_data:
                enabled_callbacks = script_data.get('enabled_callbacks') or []
            else:
                enabled_callbacks = None

            script_config = ScriptConfig(
                path=script_data.get('path', ''),
                args=script_data.get('args', []),
                depends_on=script_data.get('depends_on', []),
                delay_sec=script_data.get('delay_sec', 0),
                time_sec=script_data.get('time_sec', 60),
                kill_signal=script_data.get('kill_signal', 15),
                monitor=script_data.get('monitor', {"cpu": True, "memory": True, "disk": True}),
                environment=script_data.get('environment', {}),
                cwd=script_data.get('cwd', config_data.get('cwd', '')),
                shutdown_patterns=script_data.get('shutdown_patterns', []),
                propagate_shutdown=bool(script_data.get('propagate_shutdown', False)),
                shutdown_grace_sec=float(script_data.get('shutdown_grace_sec', 0.0) or 0.0),
                enabled_callbacks=enabled_callbacks,
                remote_env=merged_remote_env,
                host_profile=host_prof
            )
            test_config.scripts.append(script_config)

        return test_config

    def get_config(self) -> Optional[TestConfig]:
        """Get the current configuration"""
        return self._config

    def validate_config(self) -> bool:
        """Validate the configuration"""
        if not self._config:
            return False

        for script in self._config.scripts:
            if not script.path:
                print("❌ Script path is empty")
                return False

        script_paths = {script.path for script in self._config.scripts}
        for script in self._config.scripts:
            for dep in script.depends_on:
                if dep not in script_paths:
                    print(f"❌ Script {script.path} depends on missing script {dep}")
                    return False

        return True

    def get_execution_order(self) -> List[List[str]]:
        """
        Compute execution order based on dependencies

        Returns:
            List[List[str]]: each sub-list contains script paths that can run in parallel
        """
        if not self._config:
            return []

        scripts = {script.path: script for script in self._config.scripts}
        resolved = set()
        order = []

        while len(resolved) < len(scripts):
            ready = []
            for path, script in scripts.items():
                if path not in resolved:
                    if all(dep in resolved for dep in script.depends_on):
                        ready.append(path)

            if not ready:
                remaining = set(scripts.keys()) - resolved
                raise ValueError(f"Circular dependency detected: {remaining}")

            order.append(ready)
            resolved.update(ready)

        return order