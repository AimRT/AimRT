#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Resource monitor for the AimRT test framework

Monitors CPU, memory, and disk usage of processes and generates detailed reports.
"""

import psutil
import threading
import time
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from datetime import datetime
import json


@dataclass
class ResourceSnapshot:
    """Resource usage snapshot"""
    timestamp: datetime
    cpu_percent: float
    memory_rss: int  # physical memory (bytes)
    memory_vms: int  # virtual memory (bytes)
    memory_percent: float
    disk_read_bytes: int
    disk_write_bytes: int
    disk_read_count: int
    disk_write_count: int


@dataclass
class ProcessMonitorData:
    """Process monitoring data"""
    pid: int
    name: str
    start_time: datetime
    end_time: Optional[datetime] = None
    snapshots: List[ResourceSnapshot] = field(default_factory=list)
    status: str = "running"
    exit_code: Optional[int] = None
    children: List[int] = field(default_factory=list)  # child process PIDs
    children_info: List[Dict[str, Any]] = field(default_factory=list)  # child process details
    total_cpu_percent: float = 0.0  # total CPU percent including children
    total_memory_rss: int = 0  # total memory RSS including children
    total_memory_percent: float = 0.0  # total memory percent including children


class ResourceMonitor:
    """Resource monitor"""

    def __init__(self, sample_interval: float = 1.0):
        """
        Initialize the resource monitor

        Args:
            sample_interval: sampling interval in seconds
        """
        self.sample_interval = sample_interval
        self._monitors: Dict[int, ProcessMonitorData] = {}
        self._monitor_threads: Dict[int, threading.Thread] = {}
        self._stop_events: Dict[int, threading.Event] = {}
        self._lock = threading.Lock()
        # Cache psutil.Process objects to avoid the issue where the first cpu_percent is always 0
        self._proc_cache: Dict[int, psutil.Process] = {}
        # Marked PIDs that have completed cpu_percent priming (first call returns 0, needs priming once)
        self._cpu_primed: set[int] = set()

    def start_monitoring(self, pid: int, name: str) -> bool:
        """
        Start monitoring the specified process

        Args:
            pid: process ID
            name: process name

        Returns:
            bool: True if monitoring started
        """
        try:
            # Check whether the process exists
            process = psutil.Process(pid)
            if not process.is_running():
                print(f"❌ Process {pid} ({name}) is not running")
                return False

            with self._lock:
                if pid in self._monitors:
                    print(f"⚠️ Process {pid} ({name}) is already being monitored")
                    return True

                # Cache and prime cpu_percent so next sample gets non-zero value
                self._proc_cache[pid] = process
                try:
                    process.cpu_percent(None)
                except Exception:
                    pass
                self._cpu_primed.add(pid)

                # Create monitor data holder
                monitor_data = ProcessMonitorData(
                    pid=pid,
                    name=name,
                    start_time=datetime.now()
                )
                self._monitors[pid] = monitor_data

                # Create stop event
                stop_event = threading.Event()
                self._stop_events[pid] = stop_event

                # Launch monitoring thread
                monitor_thread = threading.Thread(
                    target=self._monitor_process,
                    args=(pid, stop_event),
                    daemon=True
                )
                self._monitor_threads[pid] = monitor_thread
                monitor_thread.start()

                print(f"🔍 Start monitoring process {pid} ({name})")
                return True

        except psutil.NoSuchProcess:
            print(f"❌ Process {pid} does not exist")
            return False
        except Exception as e:
            print(f"❌ Failed to start monitoring: {e}")
            return False

    def stop_monitoring(self, pid: int) -> Optional[ProcessMonitorData]:
        """
        Stop monitoring the specified process

        Args:
            pid: process ID

        Returns:
            ProcessMonitorData: monitor data, or None if the process was not monitored
        """
        with self._lock:
            if pid not in self._monitors:
                return None

            # Stop monitoring thread
            if pid in self._stop_events:
                self._stop_events[pid].set()

            if pid in self._monitor_threads:
                self._monitor_threads[pid].join(timeout=2.0)
                del self._monitor_threads[pid]

            if pid in self._stop_events:
                del self._stop_events[pid]

            # Get monitor data
            monitor_data = self._monitors.pop(pid)
            monitor_data.end_time = datetime.now()

            # Get final status
            try:
                process = psutil.Process(pid)
                if process.is_running():
                    monitor_data.status = "running"
                else:
                    monitor_data.status = "terminated"
                    monitor_data.exit_code = process.returncode
            except psutil.NoSuchProcess:
                monitor_data.status = "terminated"

            print(f"⏹️ Stop monitoring process {pid} ({monitor_data.name})")
            return monitor_data

    def stop_all_monitoring(self) -> Dict[int, ProcessMonitorData]:
        """
        Stop all monitoring

        Returns:
            Dict[int, ProcessMonitorData]: all collected monitor data
        """
        all_data = {}
        pids = list(self._monitors.keys())

        for pid in pids:
            data = self.stop_monitoring(pid)
            if data:
                all_data[pid] = data

        return all_data

    def _monitor_process(self, pid: int, stop_event: threading.Event):
        """Monitor resource usage for a process and its child processes"""
        try:
            # Reuse cached psutil.Process to avoid first-sample cpu_percent being 0
            process = self._proc_cache.get(pid)
            if process is None:
                process = psutil.Process(pid)
                self._proc_cache[pid] = process

            while not stop_event.is_set():
                try:
                    # Check process is still running
                    if not process.is_running():
                        break

                    # Get process and all of its children
                    all_processes = self._get_process_tree(pid)

                    # Compute total resource usage
                    total_cpu_percent = 0.0
                    total_memory_rss = 0
                    total_memory_percent = 0.0
                    total_disk_read_bytes = 0
                    total_disk_write_bytes = 0
                    total_disk_read_count = 0
                    total_disk_write_count = 0

                    # Get main process resources (with priming logic)
                    if pid in self._cpu_primed:
                        cpu_percent = process.cpu_percent(None)
                    else:
                        process.cpu_percent(None)
                        self._cpu_primed.add(pid)
                        cpu_percent = 0.0
                    memory_info = process.memory_info()
                    memory_percent = process.memory_percent()

                    total_cpu_percent += cpu_percent
                    total_memory_rss += memory_info.rss
                    total_memory_percent += memory_percent

                    # Get main process disk I/O
                    try:
                        io_counters = process.io_counters()
                        disk_read_bytes = io_counters.read_bytes
                        disk_write_bytes = io_counters.write_bytes
                        disk_read_count = io_counters.read_count
                        disk_write_count = io_counters.write_count
                        total_disk_read_bytes += disk_read_bytes
                        total_disk_write_bytes += disk_write_bytes
                        total_disk_read_count += disk_read_count
                        total_disk_write_count += disk_write_count
                    except (psutil.AccessDenied, AttributeError):
                        disk_read_bytes = 0
                        disk_write_bytes = 0
                        disk_read_count = 0
                        disk_write_count = 0

                    # Get child processes resources
                    child_pids = []
                    child_cpu_map: Dict[int, float] = {}
                    for child_pid in all_processes:
                        if child_pid != pid:  # skip main process
                            try:
                                child_process = self._proc_cache.get(child_pid)
                                if child_process is None:
                                    child_process = psutil.Process(child_pid)
                                    self._proc_cache[child_pid] = child_process
                                if child_process.is_running():
                                    child_pids.append(child_pid)

                                    # Child CPU usage (with priming)
                                    if child_pid in self._cpu_primed:
                                        child_cpu = child_process.cpu_percent(None)
                                    else:
                                        child_process.cpu_percent(None)
                                        self._cpu_primed.add(child_pid)
                                        child_cpu = 0.0
                                    total_cpu_percent += child_cpu
                                    child_cpu_map[child_pid] = child_cpu

                                    # Child memory usage
                                    child_memory = child_process.memory_info()
                                    child_memory_percent = child_process.memory_percent()
                                    total_memory_rss += child_memory.rss
                                    total_memory_percent += child_memory_percent

                                    # Child disk I/O
                                    try:
                                        child_io = child_process.io_counters()
                                        total_disk_read_bytes += child_io.read_bytes
                                        total_disk_write_bytes += child_io.write_bytes
                                        total_disk_read_count += child_io.read_count
                                        total_disk_write_count += child_io.write_count
                                    except (psutil.AccessDenied, AttributeError):
                                        pass

                            except (psutil.NoSuchProcess, psutil.AccessDenied):
                                continue

                    # Create snapshot (total resource usage)
                    snapshot = ResourceSnapshot(
                        timestamp=datetime.now(),
                        cpu_percent=total_cpu_percent,  # total CPU percent
                        memory_rss=total_memory_rss,    # total memory RSS
                        memory_vms=memory_info.vms,     # main process virtual memory
                        memory_percent=total_memory_percent,  # total memory percent
                        disk_read_bytes=total_disk_read_bytes,
                        disk_write_bytes=total_disk_write_bytes,
                        disk_read_count=total_disk_read_count,
                        disk_write_count=total_disk_write_count
                    )


                    # Update monitor data
                    with self._lock:
                        if pid in self._monitors:
                            monitor_data = self._monitors[pid]
                            monitor_data.snapshots.append(snapshot)
                            monitor_data.children = []
                            monitor_data.children_info = []
                            monitor_data.total_cpu_percent = total_cpu_percent
                            monitor_data.total_memory_rss = total_memory_rss
                            monitor_data.total_memory_percent = total_memory_percent

                    # Wait for next sample
                    stop_event.wait(self.sample_interval)

                except psutil.NoSuchProcess:
                    break
                except psutil.AccessDenied:
                    print(f"⚠️ No permission to access resource info of process {pid}")
                    break
                except Exception as e:
                    print(f"⚠️ Error monitoring process {pid}: {e}")
                    time.sleep(self.sample_interval)

        except Exception as e:
            print(f"❌ Monitoring thread exception: {e}")

    def _get_process_tree(self, pid: int) -> List[int]:
        """
        Get PIDs for the process and all of its descendants

        Args:
            pid: root process PID

        Returns:
            List[int]: list containing the root and all child PIDs
        """
        try:
            # Return the PID itself and its recursive children PIDs
            all_pids = [pid]  # include main process

            # Recursively collect all children
            def get_children(parent_pid: int):
                try:
                    parent = psutil.Process(parent_pid)
                    children = parent.children(recursive=True)
                    for child in children:
                        all_pids.append(child.pid)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass

            get_children(pid)
            return all_pids

        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return [pid]

    def get_monitor_data(self, pid: int) -> Optional[ProcessMonitorData]:
        """Get monitor data for a given PID"""
        with self._lock:
            return self._monitors.get(pid)

    def get_all_monitor_data(self) -> Dict[int, ProcessMonitorData]:
        """Get all monitor data"""
        with self._lock:
            return self._monitors.copy()

    def get_children_info(self, pid: int) -> List[Dict[str, Any]]:
        """
        Get child process details for the specified PID

        Args:
            pid: root process PID

        Returns:
            List[Dict[str, Any]]: list of child process info
        """
        with self._lock:
            monitor_data = self._monitors.get(pid)
            if not monitor_data:
                return []

        # Return stored child details
        return monitor_data.children_info

    def generate_report(self, monitor_data: ProcessMonitorData) -> Dict[str, Any]:
        """
        Generate a monitoring report

        Args:
            monitor_data: monitor data

        Returns:
            Dict[str, Any]: detailed monitoring report
        """
        if not monitor_data.snapshots:
            return {
                "pid": monitor_data.pid,
                "name": monitor_data.name,
                "status": "no_data",
                "message": "No monitoring data was collected"
            }

        snapshots = monitor_data.snapshots

        # Compute stats
        cpu_values = [s.cpu_percent for s in snapshots]
        memory_rss_values = [s.memory_rss for s in snapshots]
        memory_percent_values = [s.memory_percent for s in snapshots]

        # Compute disk I/O deltas (first snapshot as baseline)
        if len(snapshots) > 1:
            disk_read_delta = snapshots[-1].disk_read_bytes - snapshots[0].disk_read_bytes
            disk_write_delta = snapshots[-1].disk_write_bytes - snapshots[0].disk_write_bytes
            disk_read_count_delta = snapshots[-1].disk_read_count - snapshots[0].disk_read_count
            disk_write_count_delta = snapshots[-1].disk_write_count - snapshots[0].disk_write_count
        else:
            disk_read_delta = 0
            disk_write_delta = 0
            disk_read_count_delta = 0
            disk_write_count_delta = 0

        # Compute duration
        end_time = monitor_data.end_time or datetime.now()
        duration = (end_time - monitor_data.start_time).total_seconds()

        report = {
            "pid": monitor_data.pid,
            "name": monitor_data.name,
            "status": monitor_data.status,
            "exit_code": monitor_data.exit_code,
            "duration_seconds": duration,
            "start_time": monitor_data.start_time.isoformat(),
            "end_time": end_time.isoformat(),
            "sample_count": len(snapshots),
            "cpu": {
                "min_percent": min(cpu_values),
                "max_percent": max(cpu_values),
                "avg_percent": sum(cpu_values) / len(cpu_values),
                "final_percent": cpu_values[-1],
                "total_percent": monitor_data.total_cpu_percent  # total CPU percent including children
            },
            "memory": {
                "min_rss_mb": min(memory_rss_values) / 1024 / 1024,
                "max_rss_mb": max(memory_rss_values) / 1024 / 1024,
                "avg_rss_mb": sum(memory_rss_values) / len(memory_rss_values) / 1024 / 1024,
                "final_rss_mb": memory_rss_values[-1] / 1024 / 1024,
                "min_percent": min(memory_percent_values),
                "max_percent": max(memory_percent_values),
                "avg_percent": sum(memory_percent_values) / len(memory_percent_values),
                "final_percent": memory_percent_values[-1],
                "total_rss_mb": monitor_data.total_memory_rss / 1024 / 1024,  # total memory including children
                "total_percent": monitor_data.total_memory_percent  # total memory percent including children
            },
            "disk": {
                "total_read_mb": disk_read_delta / 1024 / 1024,
                "total_write_mb": disk_write_delta / 1024 / 1024,
                "total_read_count": disk_read_count_delta,
                "total_write_count": disk_write_count_delta,
                "avg_read_mb_per_sec": (disk_read_delta / 1024 / 1024) / duration if duration > 0 else 0,
                "avg_write_mb_per_sec": (disk_write_delta / 1024 / 1024) / duration if duration > 0 else 0
            }
        }

        return report

    def export_detailed_data(self, monitor_data: ProcessMonitorData, format: str = "json") -> str:
        """
        Export detailed monitoring data

        Args:
            monitor_data: monitor data
            format: export format ("json" or "csv")

        Returns:
            str: exported data string
        """
        if format == "json":
            data = {
                "process_info": {
                    "pid": monitor_data.pid,
                    "name": monitor_data.name,
                    "start_time": monitor_data.start_time.isoformat(),
                    "end_time": monitor_data.end_time.isoformat() if monitor_data.end_time else None,
                    "status": monitor_data.status,
                    "exit_code": monitor_data.exit_code
                },
                "snapshots": [
                    {
                        "timestamp": s.timestamp.isoformat(),
                        "cpu_percent": s.cpu_percent,
                        "memory_rss_mb": s.memory_rss / 1024 / 1024,
                        "memory_vms_mb": s.memory_vms / 1024 / 1024,
                        "memory_percent": s.memory_percent,
                        "disk_read_mb": s.disk_read_bytes / 1024 / 1024,
                        "disk_write_mb": s.disk_write_bytes / 1024 / 1024,
                        "disk_read_count": s.disk_read_count,
                        "disk_write_count": s.disk_write_count
                    }
                    for s in monitor_data.snapshots
                ]
            }
            return json.dumps(data, indent=2, ensure_ascii=False)

        elif format == "csv":
            lines = [
                "timestamp,cpu_percent,memory_rss_mb,memory_vms_mb,memory_percent,disk_read_mb,disk_write_mb,disk_read_count,disk_write_count"
            ]
            for s in monitor_data.snapshots:
                lines.append(
                    f"{s.timestamp.isoformat()},"
                    f"{s.cpu_percent},"
                    f"{s.memory_rss / 1024 / 1024:.2f},"
                    f"{s.memory_vms / 1024 / 1024:.2f},"
                    f"{s.memory_percent:.2f},"
                    f"{s.disk_read_bytes / 1024 / 1024:.2f},"
                    f"{s.disk_write_bytes / 1024 / 1024:.2f},"
                    f"{s.disk_read_count},"
                    f"{s.disk_write_count}"
                )
            return "\n".join(lines)

        else:
            raise ValueError(f"Unsupported export format: {format}")