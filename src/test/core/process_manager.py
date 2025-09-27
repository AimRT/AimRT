#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AimRT Test Framework Process Manager

Responsible for executing scripts, managing process dependencies, handling timeout control, and integrating resource monitoring functionality.
"""

import subprocess
import threading
import time
import os
import json
import pty
import select

import psutil
from typing import Dict, List, Optional, Any, Literal
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from fabric import Connection


from .config_manager import ScriptConfig
from .resource_monitor import ResourceMonitor, ProcessMonitorData, ResourceSnapshot
from .callback_manager import CallbackManager, CallbackTrigger


@dataclass
class ProcessInfo:
    """Process information"""
    script_path: str
    pid: int
    process: subprocess.Popen
    start_time: datetime
    end_time: Optional[datetime] = None
    exit_code: Optional[int] = None
    status: Literal["running", "completed", "failed", "timeout", "killed"] = "running"
    stdout: str = ""
    stderr: str = ""
    monitor_data: Optional[ProcessMonitorData] = None
    terminated_by_framework: bool = False
    backend: Literal["local", "remote"] = "local"
    host: str = ""
    remote_dir: str = ""
    remote_stdout_path: str = ""
    remote_stderr_path: str = ""
    remote_exit_code_path: str = ""
    remote_conn_key: str = ""
    shutdown_triggered: bool = False
    # Remote resource monitoring file path
    remote_monitor_script: str = ""
    remote_monitor_output: str = ""
    remote_monitor_pid_path: str = ""
    # Remote real business process PID and path, and outer session/process group PID
    remote_run_pid_path: str = ""
    remote_group_pid: int = 0
    # Local PTY support
    pty_master_fd: int = -1
    pty_reader_started: bool = False
    pty_reader_thread: Optional[threading.Thread] = None


class ProcessManager:
    """Process manager"""

    def __init__(self, base_cwd: str = "", resource_monitor: Optional[ResourceMonitor] = None,
                 callback_manager: Optional[CallbackManager] = None,
                 global_shutdown_patterns: Optional[List[str]] = None,
                 stop_all_on_shutdown: bool = False,
                 echo_child_output: bool = True):
        """
        Initialize process manager

        Args:
            base_cwd: base working directory
            resource_monitor: resource monitor instance
            callback_manager: callback manager instance
        """
        self.base_cwd = Path(base_cwd) if base_cwd else Path.cwd()
        self.resource_monitor = resource_monitor or ResourceMonitor()
        self.callback_manager = callback_manager or CallbackManager()
        self._processes: Dict[str, ProcessInfo] = {}
        self._script_configs: Dict[str, ScriptConfig] = {}
        self._lock = threading.Lock()
        self._fabric_conns: Dict[str, Any] = {}
        # Global shutdown configuration
        self._global_shutdown_patterns = [p.lower() for p in (global_shutdown_patterns or [])]
        self._stop_all_on_shutdown = bool(stop_all_on_shutdown)
        self._global_shutdown_initiated = threading.Event()
        # Whether to echo child process output to current console (for local PTY reader)
        self._echo_child_output = bool(echo_child_output)

    def _graceful_terminate_local(self, process_info: ProcessInfo):
        try:
            parent = psutil.Process(process_info.pid)
            children = parent.children(recursive=True)
            for ch in children:
                try:
                    ch.terminate()
                except psutil.NoSuchProcess:
                    pass
            psutil.wait_procs(children, timeout=3)
            try:
                parent.terminate()
            except psutil.NoSuchProcess:
                pass
            psutil.wait_procs([parent], timeout=3)
        except Exception:
            pass

    def _graceful_terminate_remote(self, process_info: ProcessInfo):
        try:
            conn = self._fabric_conns.get(process_info.remote_conn_key)
            if not conn:
                with self._lock:
                    scfg = self._script_configs.get(process_info.script_path)
                conn = self._get_fabric_connection(scfg)
            group_pid = process_info.remote_group_pid or process_info.pid
            run_pid = process_info.pid
            # Recursively graceful terminate the entire child process tree (avoid killing only the parent process)
            cmd = (
                "bash -lc '"
                "kill_tree() { local p=$1; if [ -z \"$p\" ] || [ \"$p\" = \"-1\" ]; then return; fi; "
                "for c in $(pgrep -P $p 2>/dev/null || true); do kill_tree $c; done; "
                "kill -TERM $p 2>/dev/null || true; }; "
                f"kill -TERM -{group_pid} 2>/dev/null || true; "
                f"kill_tree {group_pid}; "
                f"if [ {run_pid} -ne {group_pid} ]; then kill_tree {run_pid}; fi; "
                "sleep 0.5; "
                "'"
            )
            conn.run(cmd, hide=True, warn=True, in_stream=False)
        except Exception:
            pass

    def _trigger_global_shutdown(self, grace_sec: float = 0.0):
        if self._global_shutdown_initiated.is_set():
            return
        self._global_shutdown_initiated.set()
        try:
            if grace_sec and grace_sec > 0:
                print(f"🛎️ Trigger global shutdown, wait for {grace_sec:.1f}s grace period...")
                time.sleep(min(grace_sec, 60.0))
        except Exception:
            pass
        # Terminate all still running processes
        with self._lock:
            targets = list(self._processes.keys())
        for sp in targets:
            try:
                if self.is_process_running(sp):
                    self.kill_process(sp, reason="global_shutdown")
            except Exception:
                pass

    def _on_shutdown_pattern_matched(
            self,
            process_info: ProcessInfo,
            script_config: ScriptConfig,
            *,
            matched_global: bool):
        # Mark completed and try graceful terminate the trigger
        if process_info.backend == "remote":
            self._graceful_terminate_remote(process_info)
        else:
            self._graceful_terminate_local(process_info)
        process_info.status = "completed"
        process_info.end_time = datetime.now()
        process_info.shutdown_triggered = True

        should_propagate = self._stop_all_on_shutdown or matched_global or bool(
            getattr(script_config, 'propagate_shutdown', False))
        if should_propagate:
            grace = 0.0
            try:
                grace = float(getattr(script_config, 'shutdown_grace_sec', 0.0) or 0.0)
            except Exception:
                grace = 0.0
            self._trigger_global_shutdown(grace)

    def _start_remote_resource_monitor(
            self,
            conn: Connection,
            process_info: "ProcessInfo",
            script_config: ScriptConfig):
        """
        Start resource monitor on remote (based on psutil), write sampled data to JSONL file.
        """
        try:
            remote_python = "python3"
            chk = conn.run(
                f"{remote_python} -c 'import psutil,sys; sys.stdout.write(psutil.__version__)'",
                hide=True,
                warn=True,
                in_stream=False)
            if not chk.ok or not (chk.stdout or "").strip():
                print("⚠️  Remote psutil not installed, skip remote resource monitor")
                return

            remote_dir = process_info.remote_dir
            monitor_script_path = f"{remote_dir}/aimrt_remote_monitor.py"
            monitor_output_path = f"{remote_dir}/monitor.jsonl"
            monitor_pid_path = f"{remote_dir}/monitor_pid"

            # Write monitor script
            monitor_py = r'''#!/usr/bin/env python3
import argparse
import json
import time
from datetime import datetime
import psutil


def collect_stats(proc, children, main_cpu_percent, children_cpu_percents):
    total_cpu = main_cpu_percent + sum(children_cpu_percents)
    total_mem_rss = 0
    total_mem_percent = 0.0
    total_read_bytes = 0
    total_write_bytes = 0
    total_read_count = 0
    total_write_count = 0
    main_vms = 0

    try:
        mem = proc.memory_info()
        mem_pct = proc.memory_percent()
        try:
            io = proc.io_counters()
            rB, wB, rC, wC = io.read_bytes, io.write_bytes, io.read_count, io.write_count
        except (psutil.Error, NotImplementedError, AttributeError):
            rB = wB = rC = wC = 0
        total_mem_rss += mem.rss
        total_mem_percent += mem_pct
        total_read_bytes += rB
        total_write_bytes += wB
        total_read_count += rC
        total_write_count += wC
        main_vms = mem.vms
    except psutil.Error:
        return None

    for ch in children:
        try:
            mem = ch.memory_info()
            mem_pct = ch.memory_percent()
            try:
                io = ch.io_counters()
                rB, wB, rC, wC = io.read_bytes, io.write_bytes, io.read_count, io.write_count
            except (psutil.Error, NotImplementedError, AttributeError):
                rB = wB = rC = wC = 0
            total_mem_rss += mem.rss
            total_mem_percent += mem_pct
            total_read_bytes += rB
            total_write_bytes += wB
            total_read_count += rC
            total_write_count += wC
        except psutil.Error:
            continue

    now = datetime.now().isoformat()
    return {
        "timestamp": now,
        "cpu_percent": total_cpu,
        "memory_rss": int(total_mem_rss),
        "memory_vms": int(main_vms),
        "memory_percent": float(total_mem_percent),
        "disk_read_bytes": int(total_read_bytes),
        "disk_write_bytes": int(total_write_bytes),
        "disk_read_count": int(total_read_count),
        "disk_write_count": int(total_write_count),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pid", type=int, required=True)
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--out", type=str, required=True)
    args = ap.parse_args()

    pid = args.pid
    interval = args.interval
    out = args.out

    try:
        proc = psutil.Process(pid)
    except psutil.NoSuchProcess:
        print(f"Process with PID {pid} not found.")
        return
    proc.cpu_percent(None)
    children = {p.pid: p for p in proc.children(recursive=True)}
    for child in children.values():
        child.cpu_percent(None)

    time.sleep(interval)

    while True:
        try:
            if not proc.is_running():
                break
            main_cpu = proc.cpu_percent(None)
            children_cpu = []
            try:
                latest_children_list = proc.children(recursive=True)
                latest_by_pid = {ch.pid: ch for ch in latest_children_list}
                for pid_key, ch in latest_by_pid.items():
                    if pid_key not in children:
                        try:
                            ch.cpu_percent(None)
                        except psutil.Error:
                            pass
                        children[pid_key] = ch
                for pid_key in list(children.keys()):
                    if pid_key not in latest_by_pid:
                        children.pop(pid_key, None)
                for child in children.values():
                    try:
                        children_cpu.append(child.cpu_percent(None))
                    except psutil.Error:
                        continue

            except psutil.Error:
                pass
            data = collect_stats(proc, children.values(), main_cpu, children_cpu)
            if data is None:
                break

            with open(out, "a", encoding="utf-8") as f:
                f.write(json.dumps(data, ensure_ascii=False) + "\n")

        except (psutil.NoSuchProcess, psutil.AccessDenied):
            break
        except Exception as e:
            print(f"An error occurred: {e}")
            pass

        time.sleep(interval)

if __name__ == "__main__":
    main()
'''
            conn.run(f"cat > {monitor_script_path} <<'PY'\n{monitor_py}\nPY", hide=True, warn=True, in_stream=False)
            conn.run(f"chmod +x {monitor_script_path}", hide=True, warn=True, in_stream=False)

            # Start monitor: prioritize monitoring real business process PID
            pid_to_monitor = process_info.pid
            try:
                if process_info.remote_run_pid_path:
                    r = conn.run(
                        f"test -s {process_info.remote_run_pid_path} && cat {process_info.remote_run_pid_path} || echo -1",
                        hide=True,
                        warn=True,
                        in_stream=False)
                    rp = int((r.stdout or "").strip())
                    if rp > 0:
                        pid_to_monitor = rp
                        process_info.pid = rp
            except Exception:
                pass

            interval = getattr(self.resource_monitor, 'sample_interval', 1.0) or 1.0
            start_cmd = (
                f"nohup setsid {remote_python} {monitor_script_path} "
                f"--pid {pid_to_monitor} --interval {interval} --out {monitor_output_path} "
                f"> {remote_dir}/monitor_stdout.log 2>&1 & echo $! > {monitor_pid_path}"
            )
            conn.run(start_cmd, hide=True, warn=True, pty=False, in_stream=False)

            process_info.remote_monitor_script = monitor_script_path
            process_info.remote_monitor_output = monitor_output_path
            process_info.remote_monitor_pid_path = monitor_pid_path
            print(f"✅ Remote resource monitor started: {monitor_output_path}")
        except Exception as e:
            print(f"⚠️  Start remote resource monitor failed: {e}")
            return

    def _is_remote(self, script_config: ScriptConfig) -> bool:
        host_profile = getattr(script_config, 'host_profile', None)
        if not host_profile:
            return False
        host = (getattr(host_profile, 'host', '') or '').strip().lower()
        return bool(host and host not in ["localhost", "127.0.0.1", "::1"])

    def _fabric_key(self, script_config: ScriptConfig) -> str:
        user = (script_config.host_profile.ssh_user or "").strip() or None
        host = (script_config.host_profile.host or "").strip()
        port = int(getattr(script_config.host_profile, 'ssh_port', 22) or 22)
        return f"{user or ''}@{host}:{port}"

    def _get_fabric_connection(self, script_config: ScriptConfig) -> Connection:

        key = self._fabric_key(script_config)
        if key in self._fabric_conns:
            return self._fabric_conns[key]

        connect_kwargs = {
            "allow_agent": False,
            "look_for_keys": False,
        }
        if getattr(script_config.host_profile, 'ssh_key', ''):
            connect_kwargs["key_filename"] = script_config.host_profile.ssh_key
            connect_kwargs["look_for_keys"] = True
        if getattr(script_config.host_profile, 'ssh_password', ''):
            connect_kwargs["password"] = script_config.host_profile.ssh_password

        user = (script_config.host_profile.ssh_user or "").strip() or None
        host = (script_config.host_profile.host or "").strip()
        port = int(getattr(script_config.host_profile, 'ssh_port', 22) or 22)

        conn = Connection(
            host=host,
            user=user,
            port=port,
            connect_timeout=10,
            connect_kwargs=connect_kwargs
        )
        try:
            conn.open()
        except Exception as e:
            raise RuntimeError(f"Cannot connect to remote {key}: {e}")

        self._fabric_conns[key] = conn
        return conn

    def execute_script(self, script_config: ScriptConfig, run_time: Optional[int] = None) -> ProcessInfo:
        """
        Execute a single script

        Args:
            script_config: script configuration
            run_time: run time (seconds), if None use script configuration

        Returns:
            ProcessInfo: process information
        """
        script_run_time = run_time or script_config.time_sec

        # Prepare execution environment
        env = os.environ.copy()
        env.update(script_config.environment)

        # Determine working directory
        if script_config.cwd:
            cwd = self.base_cwd / script_config.cwd
        else:
            cwd = self.base_cwd

        # Build command
        cmd = [script_config.path] + script_config.args

        print(f"🚀 Execute script: {script_config.path}")
        print(f"    Working directory: {cwd}")
        print(f"    Run time: {script_run_time} seconds")
        print(f"    Environment variables: {script_config.environment}")

        if not self._is_remote(script_config):
            try:
                master_fd, slave_fd = pty.openpty()
                process = subprocess.Popen(
                    cmd,
                    cwd=str(cwd),
                    env=env,
                    stdin=slave_fd,
                    stdout=slave_fd,
                    stderr=slave_fd,
                    close_fds=True
                )
                try:
                    os.close(slave_fd)
                except Exception:
                    pass
                process_info = ProcessInfo(
                    script_path=script_config.path,
                    pid=process.pid,
                    process=process,
                    start_time=datetime.now(),
                    backend="local",
                    pty_master_fd=master_fd
                )

                with self._lock:
                    self._processes[script_config.path] = process_info
                    self._script_configs[script_config.path] = script_config

                monitor_enabled = any(script_config.monitor.values())
                print(f"📊 Monitor configuration: {script_config.monitor}, monitor enabled: {monitor_enabled}")
                if monitor_enabled:
                    success = self.resource_monitor.start_monitoring(process.pid, script_config.path)
                    if success:
                        print(f"✅ Resource monitor started: {script_config.path} (PID: {process.pid})")
                    else:
                        print(f"❌ Resource monitor start failed: {script_config.path} (PID: {process.pid})")

                print(f"✅ Script started successfully, PID: {process.pid}")

                context = {
                    'process_info': process_info,
                    'script_config': script_config
                }
                self.callback_manager.execute_callbacks(CallbackTrigger.PROCESS_START, context)

                self.start_periodic_callbacks()

                self._start_local_pty_reader(process_info, script_config)

                if script_run_time > 0:
                    self._start_run_time_control(process_info, script_run_time, script_config.kill_signal)

                return process_info

            except Exception as e:
                print(f"❌ Start script failed: {e}")
                process_info = ProcessInfo(
                    script_path=script_config.path,
                    pid=-1,
                    process=None,
                    start_time=datetime.now(),
                    end_time=datetime.now(),
                    status="failed",
                    stderr=str(e),
                    backend="local"
                )
                return process_info
        else:
            # Remote execution
            try:
                conn = self._get_fabric_connection(script_config)

                # Securely create a unique remote temp directory (0700) to avoid using publicly writable paths
                mktemp_res = conn.run('mktemp -d -t aimrt.XXXXXXXX', hide=True, warn=True, in_stream=False)
                remote_dir = (mktemp_res.stdout or "").strip()
                if not remote_dir:
                    # Fallback: create a directory under user's home with restrictive permissions
                    fallback_cmd = 'dir="${HOME}/.aimrt/run.$(date +%s).$$"; mkdir -p -m 700 "$dir" && printf "%s" "$dir"'
                    fb_res = conn.run(fallback_cmd, hide=True, warn=True, in_stream=False)
                    remote_dir = (fb_res.stdout or "").strip()
                    if not remote_dir:
                        raise RuntimeError("Failed to create secure remote temporary directory")

                remote_cwd = script_config.host_profile.remote_cwd or script_config.cwd or "."

                remote_env = dict(script_config.environment or {})
                remote_env.update(script_config.remote_env or {})
                env_exports = " ".join([f'export {k}="{v}";' for k, v in remote_env.items()])
                joined_cmd = " ".join(cmd)
                main_script_path = script_config.path

                stdout_path = f"{remote_dir}/stdout.log"
                stderr_path = f"{remote_dir}/stderr.log"
                pid_path = f"{remote_dir}/pid"  # Outer bash (session/process group) PID
                run_pid_path = f"{remote_dir}/run_pid"  # Real business process PID
                exit_path = f"{remote_dir}/exit_code"

                remote_shell_cmd = (
                    f"mkdir -p {remote_dir} && cd {remote_cwd} && "
                    f"if [ ! -d {remote_cwd} ]; then echo 'Remote cwd not exists: {remote_cwd}' > {stderr_path}; exit 1; fi; "
                    f"nohup setsid bash -lc '"
                    f"source /etc/profile >/dev/null 2>&1; "
                    f"[ -f ~/.bashrc ] && source ~/.bashrc >/dev/null 2>&1; "
                    f"[ -f ~/.profile ] && source ~/.profile >/dev/null 2>&1; "
                    f"{env_exports} "
                    f"printenv | sort > {remote_dir}/env.log; "
                    f"if [ -f {main_script_path} ]; then chmod +x {main_script_path} || true; fi; "
                    f"({joined_cmd}) & child=$!; echo $child > {run_pid_path}; wait $child; code=$?; echo $code > {exit_path}"
                    f"' > {stdout_path} 2> {stderr_path} < /dev/null & echo $! > {pid_path}")
                port = int(getattr(script_config.host_profile, 'ssh_port', 22) or 22)
                print(f"🌐 Remote execution: {script_config.host_profile.host}:{port}")
                print(f"    Remote working directory: {remote_cwd}")
                if remote_env:
                    print(f"    Remote environment variables: {remote_env}")
                print(f"    Command: {joined_cmd}")
                print(f"    Environment dump: {remote_dir}/env.log")
                res = conn.run(remote_shell_cmd, hide=True, warn=True, pty=False, in_stream=False)
                pid_out = conn.run(f"test -s {pid_path} && cat {pid_path} || echo -1",
                                   hide=True, warn=True, in_stream=False)
                try:
                    pid = int((pid_out.stdout or "").strip())
                except Exception:
                    pid = -1

                run_pid = -1
                if pid > 0:
                    for _ in range(10):
                        try:
                            r = conn.run(
                                f"test -s {run_pid_path} && cat {run_pid_path} || echo -1",
                                hide=True,
                                warn=True,
                                in_stream=False)
                            run_pid = int((r.stdout or "").strip())
                            if run_pid > 0:
                                break
                        except Exception:
                            pass
                        time.sleep(0.05)

                if pid <= 0:
                    msg_parts = ["Remote start failed"]
                    try:
                        chk = conn.run(f"test -d {remote_cwd}", hide=True, warn=True, in_stream=False)
                        if not chk.ok:
                            msg_parts.append(f"Directory not exists: {remote_cwd}")
                    except Exception:
                        pass
                    try:
                        err_tail = conn.run(
                            f"tail -n 50 {stderr_path} 2>/dev/null || true",
                            hide=True,
                            warn=True,
                            in_stream=False).stdout
                        if err_tail:
                            msg_parts.append("stderr tail:\n" + err_tail)
                    except Exception:
                        pass
                    if res is not None and hasattr(res, 'stderr') and res.stderr:
                        msg_parts.append("run stderr:\n" + res.stderr)

                    process_info = ProcessInfo(
                        script_path=script_config.path,
                        pid=-1,
                        process=None,
                        start_time=datetime.now(),
                        end_time=datetime.now(),
                        status="failed",
                        stderr="\n".join(msg_parts),
                        backend="remote",
                        host=script_config.host_profile.host,
                        remote_dir=remote_dir,
                        remote_stdout_path=stdout_path,
                        remote_stderr_path=stderr_path,
                        remote_exit_code_path=exit_path,
                        remote_conn_key=self._fabric_key(script_config)
                    )

                    with self._lock:
                        self._processes[script_config.path] = process_info
                        self._script_configs[script_config.path] = script_config

                    context = {
                        'process_info': process_info,
                        'script_config': script_config
                    }
                    self.callback_manager.execute_callbacks(CallbackTrigger.PROCESS_START, context)
                    self.start_periodic_callbacks()
                    return process_info

                effective_pid = run_pid if run_pid > 0 else pid
                process_info = ProcessInfo(
                    script_path=script_config.path,
                    pid=effective_pid,
                    process=None,
                    start_time=datetime.now(),
                    backend="remote",
                    host=script_config.host_profile.host,
                    remote_dir=remote_dir,
                    remote_stdout_path=stdout_path,
                    remote_stderr_path=stderr_path,
                    remote_exit_code_path=exit_path,
                    remote_conn_key=self._fabric_key(script_config),
                    remote_run_pid_path=run_pid_path,
                    remote_group_pid=pid
                )

                with self._lock:
                    self._processes[script_config.path] = process_info
                    self._script_configs[script_config.path] = script_config

                monitor_enabled = any(script_config.monitor.values())
                if monitor_enabled:
                    self._start_remote_resource_monitor(conn, process_info, script_config)

                print(f"✅ Remote script started, PID: {pid}, remote directory: {remote_dir}")

                context = {
                    'process_info': process_info,
                    'script_config': script_config
                }
                self.callback_manager.execute_callbacks(CallbackTrigger.PROCESS_START, context)

                self.start_periodic_callbacks()

                if script_config.shutdown_patterns or self._global_shutdown_patterns:
                    self._start_remote_shutdown_watcher(process_info, script_config)

                # Run time control
                if script_run_time > 0:
                    self._start_run_time_control(process_info, script_run_time, script_config.kill_signal)

                return process_info

            except Exception as e:
                print(f"❌ Start remote script failed: {e}")
                process_info = ProcessInfo(
                    script_path=script_config.path,
                    pid=-1,
                    process=None,
                    start_time=datetime.now(),
                    end_time=datetime.now(),
                    status="failed",
                    stderr=str(e),
                    backend="remote",
                    host=script_config.host_profile.host
                )
                return process_info

    def wait_for_process(self, script_path: str, timeout: Optional[int] = None) -> ProcessInfo:
        """
        Wait for process to complete

        Args:
            script_path: script path
            timeout: timeout (seconds), None means no timeout wait

        Returns:
            ProcessInfo: updated process information
        """
        with self._lock:
            process_info = self._processes.get(script_path)

        if not process_info:
            print(f"❌ Script process not found: {script_path}")
            return process_info

        print(f"⏳ Wait for script to complete: {script_path}")

        try:
            if process_info.backend == "remote":
                # Remote wait logic
                try:
                    conn = self._fabric_conns.get(process_info.remote_conn_key)
                    if not conn:
                        # Try to restore connection
                        with self._lock:
                            scfg = self._script_configs.get(script_path)
                        conn = self._get_fabric_connection(scfg)
                except Exception as e:
                    raise RuntimeError(f"Remote connection lost: {e}")

                def remote_running() -> bool:
                    r = conn.run(f"ps -p {process_info.pid} -o pid=", hide=True, warn=True, in_stream=False)
                    return bool((r.stdout or "").strip())

                start_wait = time.time()
                while True:
                    if not remote_running():
                        break
                    if timeout is not None and (time.time() - start_wait) > timeout:
                        print(f"⏰ Script execution timeout (remote): {script_path}")
                        process_info.status = "timeout"
                        process_info.end_time = datetime.now()
                        self.kill_process(script_path)
                        break
                    time.sleep(0.2)

                try:
                    stdout = conn.run(f"cat {process_info.remote_stdout_path}",
                                      hide=True, warn=True, in_stream=False).stdout
                except Exception:
                    stdout = ""
                try:
                    stderr = conn.run(f"cat {process_info.remote_stderr_path}",
                                      hide=True, warn=True, in_stream=False).stdout
                except Exception:
                    stderr = ""
                process_info.stdout = (process_info.stdout or "") + (stdout or "")
                process_info.stderr = (process_info.stderr or "") + (stderr or "")

                exit_code = None
                try:
                    rc = conn.run(f"cat {process_info.remote_exit_code_path}", hide=True, warn=True, in_stream=False)
                    txt = (rc.stdout or "").strip()
                    if txt:
                        exit_code = int(txt)
                except Exception:
                    exit_code = None

                process_info.end_time = datetime.now()
                process_info.exit_code = exit_code

                try:
                    if process_info.remote_monitor_output:
                        out = conn.run(
                            f"test -f {process_info.remote_monitor_output} && cat {process_info.remote_monitor_output} || true",
                            hide=True,
                            warn=True,
                            in_stream=False)
                        lines = [ln for ln in (out.stdout or "").splitlines() if ln.strip()]
                        if lines:
                            pm = ProcessMonitorData(
                                pid=process_info.pid,
                                name=process_info.script_path,
                                start_time=process_info.start_time,
                                end_time=process_info.end_time,
                                status="terminated"
                            )
                            for ln in lines:
                                try:
                                    rec = json.loads(ln)
                                except Exception:
                                    continue
                                try:
                                    ts = datetime.fromisoformat(rec.get("timestamp"))
                                except Exception:
                                    ts = datetime.now()
                                snap = ResourceSnapshot(
                                    timestamp=ts,
                                    cpu_percent=float(rec.get("cpu_percent", 0.0)),
                                    memory_rss=int(rec.get("memory_rss", 0)),
                                    memory_vms=int(rec.get("memory_vms", 0)),
                                    memory_percent=float(rec.get("memory_percent", 0.0)),
                                    disk_read_bytes=int(rec.get("disk_read_bytes", 0)),
                                    disk_write_bytes=int(rec.get("disk_write_bytes", 0)),
                                    disk_read_count=int(rec.get("disk_read_count", 0)),
                                    disk_write_count=int(rec.get("disk_write_count", 0)),
                                )
                                pm.snapshots.append(snap)
                            if pm.snapshots:
                                last = pm.snapshots[-1]
                                pm.total_cpu_percent = last.cpu_percent
                                pm.total_memory_rss = last.memory_rss
                                pm.total_memory_percent = last.memory_percent
                            process_info.monitor_data = pm
                            print(f"📊 Collected remote monitor data: {len(pm.snapshots)} samples")
                except Exception as e:
                    print(f"⚠️ Read remote monitor data failed: {e}")

                if process_info.status in ["killed", "timeout", "completed"] or process_info.shutdown_triggered:
                    # Already marked by watcher or graceful exit triggered, do not overwrite to failed
                    if process_info.status in ["killed", "timeout"]:
                        label = "killed" if process_info.status == "killed" else "timeout"
                        print(f"⏹️ Script {label} (remote): {script_path}")
                    else:
                        print(f"✅ Remote script execution completed: {script_path}")
                else:
                    if exit_code is None:
                        process_info.status = "completed" if not remote_running() else "failed"
                    else:
                        process_info.status = "completed" if exit_code == 0 else "failed"
                    status_emoji = "✅" if process_info.status == "completed" else "❌"
                    print(f"{status_emoji} Remote script execution {process_info.status}: {script_path}")

            else:
                if not process_info.process:
                    print(f"❌ Local script process not found: {script_path}")
                    return process_info
                try:
                    with self._lock:
                        scfg = self._script_configs.get(script_path)
                    has_shutdown_watcher = bool(scfg and getattr(scfg, 'shutdown_patterns', None))
                except Exception:
                    has_shutdown_watcher = False

                use_pty = getattr(process_info, 'pty_master_fd', -1) >= 0

                if has_shutdown_watcher or use_pty:
                    if timeout is not None:
                        process_info.process.wait(timeout=timeout)
                    else:
                        process_info.process.wait()

                    process_info.end_time = datetime.now()
                    process_info.exit_code = process_info.process.returncode
                    # Ensure PTY reader drains remaining output before returning
                    try:
                        t = getattr(process_info, 'pty_reader_thread', None)
                        if t and t.is_alive():
                            t.join(timeout=1.0)
                    except Exception:
                        pass
                else:
                    if timeout is not None:
                        stdout, stderr = process_info.process.communicate(timeout=timeout)
                    else:
                        stdout, stderr = process_info.process.communicate()

                    process_info.end_time = datetime.now()
                    process_info.exit_code = process_info.process.returncode
                    process_info.stdout = (process_info.stdout or "") + (stdout or "")
                    process_info.stderr = (process_info.stderr or "") + (stderr or "")

                # If previously marked as killed/timeout/completed (e.g. set by shutdown
                # watcher), respect the existing state, avoid overwriting
                if process_info.status in ["killed", "timeout", "completed"] or process_info.shutdown_triggered:
                    label = "Killed" if process_info.status == "killed" else "Timeout"
                    if process_info.status in ["killed", "timeout"]:
                        print(f"⏹️ Script {label}: {script_path}")
                else:
                    if process_info.exit_code == 0:
                        process_info.status = "completed"
                        print(f"✅ Script execution successful: {script_path}")
                    else:
                        process_info.status = "failed"
                        print(f"❌ Script execution failed: {script_path}, exit code: {process_info.exit_code}")

        except subprocess.TimeoutExpired:
            print(f"⏰ Script execution timeout: {script_path}")
            process_info.status = "timeout"
            process_info.end_time = datetime.now()

            # Terminate process
            self.kill_process(script_path)

            # Collect remaining logs after timeout
            try:
                if process_info.process:
                    stdout, stderr = process_info.process.communicate(timeout=1)
                    process_info.stdout = (process_info.stdout or "") + (stdout or "")
                    process_info.stderr = (process_info.stderr or "") + (stderr or "")
            except Exception:
                pass

        except Exception as e:
            print(f"❌ Error waiting for script: {e}")
            process_info.status = "failed"
            process_info.end_time = datetime.now()
            process_info.stderr = str(e)

        # Stop resource monitor
        if process_info.pid > 0 and process_info.backend == "local":
            print(f"🛑 Stop resource monitor: {script_path} (PID: {process_info.pid})")
            monitor_data = self.resource_monitor.stop_monitoring(process_info.pid)
            if monitor_data:
                process_info.monitor_data = monitor_data
                print(f"📊 Collected monitor data: {len(monitor_data.snapshots)} samples")
            else:
                print(f"⚠️ No monitor data collected: {script_path}")

        with self._lock:
            scfg = self._script_configs.get(script_path)
        context = {
            'process_info': process_info,
            'monitor_data': process_info.monitor_data,
            'script_config': scfg
        }
        self.callback_manager.execute_callbacks(CallbackTrigger.PROCESS_END, context)

        return process_info

    def kill_process(self, script_path: str, *, reason: str = "") -> bool:
        """
        Force terminate process and all its child processes

        Args:
            script_path: script path

        Returns:
            bool: True on success
        """
        with self._lock:
            process_info = self._processes.get(script_path)

        if not process_info:
            return False

        try:
            if process_info.backend == "remote":
                try:
                    conn = self._fabric_conns.get(process_info.remote_conn_key)
                    if not conn:
                        with self._lock:
                            scfg = self._script_configs.get(script_path)
                        conn = self._get_fabric_connection(scfg)
                except Exception as e:
                    print(f"❌ Remote connection unavailable, cannot terminate: {e}")
                    return False

                print(
                    f"🔪 Force terminate remote process and its child processes: {script_path} (PID: {process_info.pid})")
                try:
                    if process_info.remote_monitor_pid_path:
                        mp = conn.run(
                            f"test -s {process_info.remote_monitor_pid_path} && cat {process_info.remote_monitor_pid_path} || echo -1",
                            hide=True,
                            warn=True,
                            in_stream=False)
                        try:
                            mpid = int((mp.stdout or "").strip())
                        except Exception:
                            mpid = -1
                        if mpid > 0:
                            conn.run(f"kill -TERM {mpid} || true", hide=True, warn=True, in_stream=False)
                except Exception:
                    pass
                group_pid = process_info.remote_group_pid or process_info.pid
                run_pid = process_info.pid
                # Recursively TERM the entire tree and process group
                term_cmd = (
                    "bash -lc '"
                    "kill_tree() { local p=$1; if [ -z \"$p\" ] || [ \"$p\" = \"-1\" ]; then return; fi; "
                    "for c in $(pgrep -P $p 2>/dev/null || true); do kill_tree $c; done; "
                    "kill -TERM $p 2>/dev/null || true; }; "
                    f"kill -TERM -{group_pid} 2>/dev/null || true; "
                    f"kill_tree {group_pid}; "
                    f"if [ {run_pid} -ne {group_pid} ]; then kill_tree {run_pid}; fi; "
                    "sleep 0.8; "
                    "'"
                )
                conn.run(term_cmd, hide=True, warn=True, in_stream=False)
                time.sleep(0.5)
                r = conn.run(f"ps -p {process_info.pid} -o pid=", hide=True, warn=True, in_stream=False)
                still_alive = bool((r.stdout or "").strip())
                used_kill = False
                if still_alive:
                    print("⚠️ Remote process not terminated, execute KILL...")
                    kill_cmd = (
                        "bash -lc '"
                        "kill_tree_k() { local p=$1; if [ -z \"$p\" ] || [ \"$p\" = \"-1\" ]; then return; fi; "
                        "for c in $(pgrep -P $p 2>/dev/null || true); do kill_tree_k $c; done; "
                        "kill -KILL $p 2>/dev/null || true; }; "
                        f"kill -KILL -{group_pid} 2>/dev/null || true; "
                        f"kill_tree_k {group_pid}; "
                        f"if [ {run_pid} -ne {group_pid} ]; then kill_tree_k {run_pid}; fi; "
                        "sleep 0.3; "
                        "'"
                    )
                    conn.run(kill_cmd, hide=True, warn=True, in_stream=False)
                    used_kill = True

                if reason == "global_shutdown" and not used_kill:
                    process_info.status = "completed"
                else:
                    process_info.status = "killed"
                process_info.end_time = datetime.now()
                process_info.terminated_by_framework = True
                return True
            else:
                if not process_info.process:
                    return False

                print(f"🔪 Force terminate process and its child processes: {script_path} (PID: {process_info.pid})")

                try:
                    parent_process = psutil.Process(process_info.pid)
                    child_processes = parent_process.children(recursive=True)
                    all_processes = child_processes

                except psutil.NoSuchProcess:
                    print(f"⚠️  Process {process_info.pid} does not exist")
                    process_info.status = "killed"
                    process_info.end_time = datetime.now()
                    process_info.terminated_by_framework = True
                    return True

                for proc in reversed(all_processes):
                    try:
                        proc.terminate()
                    except psutil.NoSuchProcess:
                        continue

                _, alive = psutil.wait_procs(all_processes, timeout=5)

                used_kill = False
                if alive:
                    print(f"⚠️  {len(alive)} processes cannot be terminated gracefully, force kill...")
                    for proc in alive:
                        try:
                            proc.kill()
                        except psutil.NoSuchProcess:
                            continue
                    psutil.wait_procs(alive, timeout=3)
                    used_kill = True

                if reason == "global_shutdown" and not used_kill:
                    process_info.status = "completed"
                else:
                    process_info.status = "killed"
                process_info.end_time = datetime.now()
                process_info.terminated_by_framework = True

                return True

        except Exception as e:
            print(f"❌ Terminate process failed: {e}")
            return False

    def execute_scripts_with_dependencies(self, scripts: List[ScriptConfig]) -> Dict[str, ProcessInfo]:
        """
        Execution logic:
        - Each script is independent, does not need to wait for the previous one to complete
        - As long as the dependent process is started, the next one can be started according to the delay time

        Args:
            scripts: script configuration list

        Returns:
            Dict[str, ProcessInfo]: execution results of all processes
        """
        print("🎯 Start parallel execution of script group")

        # Build script mapping (for dependency check)
        results = {}
        started_scripts = set()  # Started scripts

        # Start threads to handle script startup
        import threading

        def start_script_when_ready(script_config: ScriptConfig):
            """Start script when dependencies are satisfied"""
            script_path = script_config.path

            # Wait for dependent scripts to start
            while True:
                dependencies_satisfied = True
                for dep_path in script_config.depends_on:
                    if dep_path not in started_scripts:
                        dependencies_satisfied = False
                        break

                if dependencies_satisfied:
                    break

                # Wait briefly and then check again
                time.sleep(0.5)

            # Wait for delay time
            if script_config.delay_sec > 0:
                print(f"⏱️ Wait {script_config.delay_sec} seconds before starting: {script_path}")
                time.sleep(script_config.delay_sec)

            # Start script
            print(f"🚀 Start script: {script_path}")
            process_info = self.execute_script(script_config)
            results[script_path] = process_info

            # Mark as started (even if startup fails, mark it to avoid blocking other scripts)
            started_scripts.add(script_path)

        # Start monitor thread for each script
        threads = []
        for script in scripts:
            thread = threading.Thread(
                target=start_script_when_ready,
                args=(script,),
                daemon=True
            )
            threads.append(thread)
            thread.start()

        # Wait for all scripts to start
        for thread in threads:
            thread.join()

        print(f"✅ All scripts have started, {len(started_scripts)} in total")

        print("⏳ Wait for all scripts to complete...")

        for script in scripts:
            script_path = script.path
            if script_path in results and results[script_path].status not in ["failed"]:
                # Wait for process to end naturally (managed by run time control thread)
                process_info = self.wait_for_process(script_path, None)
                results[script_path] = process_info

                status_emoji = "✅" if process_info.status == "completed" else "❌"
                print(f"{status_emoji} Script {script_path} execution {process_info.status}")

        return results

    def register_callback(self, callback):
        """Register custom callback"""
        self.callback_manager.register_callback(callback)

    def register_function_callback(self, name: str, trigger: CallbackTrigger,
                                   func, **kwargs):
        """Register function callback"""
        return self.callback_manager.register_function_callback(name, trigger, func, **kwargs)

    def get_callback_results(self, callback_name: Optional[str] = None):
        """Get callback execution results"""
        return self.callback_manager.get_callback_results(callback_name)

    def start_periodic_callbacks(self):
        """Start periodic callbacks"""
        def context_provider():
            # Provide context for all processes
            with self._lock:
                processes = list(self._processes.values())

            return {
                'all_processes': processes,
                'active_processes': [p for p in processes if p.status == "running"]
            }

        self.callback_manager.start_periodic_callbacks(context_provider)

    def stop_periodic_callbacks(self):
        """Stop periodic callbacks"""
        self.callback_manager.stop_periodic_callbacks()

    def _register_script_callbacks(self, script_config: ScriptConfig):
        """Disabled: YAML callback registration (only keep function callback API)."""
        return

    def _start_run_time_control(self, process_info: ProcessInfo, run_time: int, kill_signal: int):
        """
        Start run time control thread

        Args:
            process_info: process information
            run_time: run time (seconds)
            kill_signal: termination signal
        """
        def run_time_controller():
            """Run time controller"""
            try:
                # Wait for specified run time
                time.sleep(run_time)

                still_running = False
                try:
                    if process_info.backend == "remote":
                        conn = self._fabric_conns.get(process_info.remote_conn_key)
                        if not conn:
                            with self._lock:
                                scfg = self._script_configs.get(process_info.script_path)
                            conn = self._get_fabric_connection(scfg)
                        r = conn.run(f"ps -p {process_info.pid} -o pid=", hide=True, warn=True, in_stream=False)
                        still_running = bool((r.stdout or "").strip())
                    else:
                        if process_info.process and process_info.process.poll() is None:
                            still_running = True
                except Exception:
                    pass

                if still_running:
                    print(f"⏰ Script {process_info.script_path} run time expired ({run_time} seconds), close process")
                    self.kill_process(process_info.script_path)

            except Exception as e:
                print(f"❌ Run time control thread exception: {e}")

        # Start control thread
        control_thread = threading.Thread(
            target=run_time_controller,
            daemon=True
        )
        control_thread.start()

    def _start_shutdown_pattern_watcher(self, process_info: ProcessInfo, script_config: ScriptConfig):
        patterns = [p.lower() for p in script_config.shutdown_patterns]

        def watcher():
            try:
                proc = process_info.process
                if not proc:
                    return
                import selectors
                sel = selectors.DefaultSelector()
                if proc.stdout:
                    sel.register(proc.stdout, selectors.EVENT_READ)
                if proc.stderr:
                    sel.register(proc.stderr, selectors.EVENT_READ)

                matched = False
                while proc.poll() is None and not matched:
                    for key, _ in sel.select(timeout=0.2):
                        data = key.fileobj.readline()
                        if not data:
                            continue
                        if key.fileobj is proc.stdout:
                            process_info.stdout += data
                            # Print local real-time standard output
                            try:
                                print(data, end="", flush=True)
                            except Exception:
                                pass
                        else:
                            process_info.stderr += data
                            # Print local real-time standard error
                            try:
                                print(data, end="", flush=True)
                            except Exception:
                                pass
                        low = data.lower()
                        for p in patterns:
                            if p in low:
                                matched = True
                                break
                        if matched:
                            break

                if matched and proc.poll() is None:
                    print(f"🛎️ Trigger shutdown_patterns, graceful terminate: {process_info.script_path}")
                    try:
                        parent = psutil.Process(process_info.pid)
                        children = parent.children(recursive=True)
                        if children is None:
                            try:
                                parent.terminate()
                            except psutil.NoSuchProcess:
                                pass
                        else:

                            for ch in children:
                                try:
                                    ch.terminate()
                                except psutil.NoSuchProcess:
                                    pass
                            psutil.wait_procs(children, timeout=3)

                        psutil.wait_procs([parent], timeout=3)
                    except Exception:
                        pass
                    process_info.status = "completed"
                    process_info.end_time = datetime.now()
                    process_info.shutdown_triggered = True
            except Exception as e:
                print(f"❌ shutdown watcher exception: {e}")

        t = threading.Thread(target=watcher, daemon=True, name=f"shutdown-watcher-{process_info.pid}")
        t.start()

    def _start_local_pty_reader(self, process_info: ProcessInfo, script_config: ScriptConfig):
        if not process_info or not process_info.process:
            return
        if getattr(process_info, 'pty_reader_started', False):
            return
        master_fd = getattr(process_info, 'pty_master_fd', -1)
        if master_fd < 0:
            return

        patterns_local = [p.lower() for p in (script_config.shutdown_patterns or [])]
        patterns_global = list(self._global_shutdown_patterns or [])

        def reader():
            try:
                rolling_low = ""
                while True:
                    try:
                        r, _, _ = select.select([master_fd], [], [], 0.2)
                    except Exception:
                        r = []
                    if master_fd in r:
                        try:
                            chunk = os.read(master_fd, 4096)
                        except OSError:
                            break
                        if not chunk:
                            break
                        try:
                            text = chunk.decode(errors="ignore")
                        except Exception:
                            text = ""
                        if text:
                            process_info.stdout = (process_info.stdout or "") + text
                            try:
                                if self._echo_child_output:
                                    print(text, end="", flush=True)
                            except Exception:
                                pass

                            if patterns_local or patterns_global:
                                rolling_low = (rolling_low + text.lower())[-8192:]
                                matched = False
                                matched_global = False
                                for p in patterns_local:
                                    if p in rolling_low:
                                        matched = True
                                        break
                                if not matched and patterns_global:
                                    for p in patterns_global:
                                        if p in rolling_low:
                                            matched = True
                                            matched_global = True
                                            break
                                if matched and process_info.process and process_info.process.poll() is None:
                                    if self._echo_child_output:
                                        print(
                                            f"\U0001F6CE\ufe0f Trigger {'global' if matched_global else 'script'}shutdown_patterns, graceful terminate (PTY): {process_info.script_path}")
                                    self._on_shutdown_pattern_matched(
                                        process_info, script_config, matched_global=matched_global)

                    if process_info.process and process_info.process.poll() is not None:
                        try:
                            while True:
                                try:
                                    chunk = os.read(master_fd, 4096)
                                except OSError:
                                    break
                                if not chunk:
                                    break
                                try:
                                    text = chunk.decode(errors="ignore")
                                except Exception:
                                    text = ""
                                if not text:
                                    break
                                process_info.stdout = (process_info.stdout or "") + text
                                try:
                                    if self._echo_child_output:
                                        print(text, end="", flush=True)
                                except Exception:
                                    pass
                        except Exception:
                            pass
                        break
            finally:
                try:
                    os.close(master_fd)
                except Exception:
                    pass
                process_info.pty_master_fd = -1

        t = threading.Thread(target=reader, daemon=True, name=f"pty-reader-{process_info.pid}")
        t.start()
        process_info.pty_reader_started = True
        process_info.pty_reader_thread = t

    def _start_remote_shutdown_watcher(self, process_info: ProcessInfo, script_config: ScriptConfig):
        patterns_local = [p.lower() for p in (script_config.shutdown_patterns or [])]
        patterns_global = list(self._global_shutdown_patterns or [])

        def watcher():
            try:
                conn = self._fabric_conns.get(process_info.remote_conn_key)
                if not conn:
                    with self._lock:
                        scfg = self._script_configs.get(script_config.path)
                    conn = self._get_fabric_connection(scfg)

                def running() -> bool:
                    r = conn.run(f"ps -p {process_info.pid} -o pid=", hide=True, warn=True, in_stream=False)
                    return bool((r.stdout or "").strip())

                matched = False
                matched_global = False
                while running() and not matched:
                    try:
                        combined = patterns_local + patterns_global
                        if not combined:
                            break
                        pattern_regex = "|".join(combined)
                        pattern_regex = pattern_regex.replace('"', '\\"')
                        out1 = conn.run(
                            f'grep -i -E "{pattern_regex}" {process_info.remote_stdout_path} 2>/dev/null || true',
                            hide=True, warn=True, in_stream=False
                        )
                        out2 = conn.run(
                            f'grep -i -E "{pattern_regex}" {process_info.remote_stderr_path} 2>/dev/null || true',
                            hide=True, warn=True, in_stream=False
                        )
                        data = (out1.stdout or "") + (out2.stdout or "")
                        low = data.lower()
                        for p in patterns_local:
                            if p in low:
                                matched = True
                                break
                        if not matched and patterns_global:
                            for p in patterns_global:
                                if p in low:
                                    matched = True
                                    matched_global = True
                                    break
                    except Exception:
                        pass
                    time.sleep(0.2)

                if matched and running():
                    print(
                        f"🛎️ Trigger remote {'global' if matched_global else 'script'}shutdown_patterns, graceful terminate: {process_info.script_path}")
                    self._on_shutdown_pattern_matched(process_info, script_config, matched_global=matched_global)
            except Exception as e:
                print(f"❌ Remote shutdown watcher exception: {e}")

        t = threading.Thread(target=watcher, daemon=True, name=f"remote-shutdown-watcher-{process_info.pid}")
        t.start()

    def get_process_info(self, script_path: str) -> Optional[ProcessInfo]:
        """Get process information"""
        with self._lock:
            return self._processes.get(script_path)

    def get_all_processes(self) -> Dict[str, ProcessInfo]:
        """Get all process information"""
        with self._lock:
            return self._processes.copy()

    def is_process_running(self, script_path: str) -> bool:
        """Check if process is running"""
        process_info = self.get_process_info(script_path)
        if not process_info:
            return False

        try:
            if process_info.backend == "remote":
                conn = self._fabric_conns.get(process_info.remote_conn_key)
                if not conn:
                    with self._lock:
                        scfg = self._script_configs.get(script_path)
                    conn = self._get_fabric_connection(scfg)
                r = conn.run(f"ps -p {process_info.pid} -o pid=", hide=True, warn=True, in_stream=False)
                return bool((r.stdout or "").strip())
            else:
                if not process_info.process:
                    return False
                return process_info.process.poll() is None
        except Exception:
            return False

    def cleanup(self):
        """Clean up all processes"""
        print("🧹 Clean up all processes...")

        with self._lock:
            script_paths = list(self._processes.keys())

        # Terminate all running processes
        for script_path in script_paths:
            if self.is_process_running(script_path):
                self.kill_process(script_path)

        # Stop all resource monitoring
        self.resource_monitor.stop_all_monitoring()

        print("✅ Process cleanup completed")

    def generate_summary_report(self) -> Dict[str, Any]:
        """Generate execution summary report"""
        processes = self.get_all_processes()

        total_count = len(processes)
        completed_count = sum(1 for p in processes.values() if p.status == "completed")
        failed_count = sum(1 for p in processes.values() if p.status == "failed")
        timeout_count = sum(1 for p in processes.values() if p.status == "timeout")
        killed_count = sum(1 for p in processes.values() if p.status == "killed")

        # Calculate total execution time
        start_times = [p.start_time for p in processes.values()]
        end_times = [p.end_time for p in processes.values() if p.end_time]

        if start_times and end_times:
            total_duration = (max(end_times) - min(start_times)).total_seconds()
        else:
            total_duration = 0

        report = {
            "summary": {
                "total_processes": total_count,
                "completed": completed_count,
                "failed": failed_count,
                "timeout": timeout_count,
                "killed": killed_count,
                "success_rate": (completed_count / total_count * 100) if total_count > 0 else 0,
                "total_duration_seconds": total_duration
            },
            "processes": {}
        }

        # Add detailed information for each process
        for script_path, process_info in processes.items():
            process_report = {
                "script_path": script_path,
                "pid": process_info.pid,
                "status": process_info.status,
                "exit_code": process_info.exit_code,
                "start_time": process_info.start_time.isoformat(),
                "end_time": process_info.end_time.isoformat() if process_info.end_time else None,
                "duration_seconds": (
                    (process_info.end_time - process_info.start_time).total_seconds()
                    if process_info.end_time else None
                ),
                "stdout_length": len(process_info.stdout),
                "stderr_length": len(process_info.stderr),
                "has_errors": bool(process_info.stderr.strip())
            }

            # Add resource monitoring report
            if process_info.monitor_data:
                process_report["resource_usage"] = self.resource_monitor.generate_report(
                    process_info.monitor_data
                )

            report["processes"][script_path] = process_report

        return report
