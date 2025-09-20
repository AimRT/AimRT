#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Report generator for the AimRT test framework

Generates detailed test reports in HTML and JSON formats.
"""

import json
from datetime import datetime
from pathlib import Path
from typing import Dict, Any
from html import escape

from .process_manager import ProcessInfo


class ReportGenerator:
    """Report generator"""

    def __init__(self, output_dir: str = "test_reports"):
        """
        Initialize the report generator

        Args:
            output_dir: report output directory
        """
        # Anchor output directory to current working directory (pytest execution CWD)
        output_path = Path(output_dir)
        if not output_path.is_absolute():
            output_path = Path.cwd() / output_path
        self.output_dir = output_path
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.html_dir = self.output_dir / "html"
        self.json_dir = self.output_dir / "json"
        self.html_dir.mkdir(parents=True, exist_ok=True)
        self.json_dir.mkdir(parents=True, exist_ok=True)

    def generate_json_report(self, test_name: str,
                           execution_results: Dict[str, ProcessInfo],
                           summary_data: Dict[str, Any],
                           pytest_results: Dict[str, Any] | None = None,
                           callback_results: Dict[str, Any] | None = None) -> str:
        """
        Generate JSON report

        Args:
            test_name: test name
            execution_results: execution results
            summary_data: summary data

        Returns:
            str: report file path
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{test_name}_{timestamp}_report.json"
        filepath = self.json_dir / filename

        # Build report payload
        report_data = {
            "test_info": {
                "name": test_name,
                "timestamp": datetime.now().isoformat(),
                "report_type": "json"
            },
            "summary": summary_data,
            "detailed_results": {}
        }

        for script_path, process_info in execution_results.items():
            process_data = {
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
                "stdout": process_info.stdout,
                "stderr": process_info.stderr
            }

            if process_info.monitor_data:
                from .resource_monitor import ResourceMonitor
                monitor = ResourceMonitor()
                resource_report = monitor.generate_report(process_info.monitor_data)
                process_data["resource_usage"] = resource_report

            report_data["detailed_results"][script_path] = process_data

        # Attach pytest results
        if pytest_results:
            report_data["pytest"] = pytest_results

        # Attach callback results (flatten for readability)
        if callback_results:
            try:
                serialized: Dict[str, Any] = {}
                for cb_name, results in (callback_results or {}).items():
                    items = []
                    for r in (results or []):
                        try:
                            items.append({
                                "success": bool(getattr(r, 'success', False)),
                                "message": str(getattr(r, 'message', '') or ''),
                                "data": getattr(r, 'data', {}) or {},
                                "warnings": getattr(r, 'warnings', []) or [],
                                "errors": getattr(r, 'errors', []) or [],
                                "timestamp": getattr(getattr(r, 'timestamp', None), 'isoformat', lambda: None)()
                            })
                        except Exception:
                            items.append({"success": False, "message": "callback result serialization error"})
                    serialized[cb_name] = items
                report_data["callbacks"] = serialized
            except Exception:
                report_data["callbacks"] = {"_error": "failed to serialize callback results"}

        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(report_data, f, indent=2, ensure_ascii=False)

        print(f"📄 JSON report generated: {filepath}")
        return str(filepath)

    def generate_html_report(self, test_name: str,
                           execution_results: Dict[str, ProcessInfo],
                           summary_data: Dict[str, Any],
                           callback_results: Dict[str, Any] | None = None,
                           pytest_results: Dict[str, Any] | None = None) -> str:
        """
        Generate HTML report

        Args:
            test_name: test name
            execution_results: execution results
            summary_data: summary data

        Returns:
            str: report file path
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{test_name}_{timestamp}_report.html"
        filepath = self.html_dir / filename

        # Inline logs directly without extra files
        html_content = self._generate_html_content(test_name, execution_results, summary_data, callback_results, pytest_results)

        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(html_content)

        print(f"🌐 HTML report generated: {filepath}")
        try:
            pytest_summary = (pytest_results or {}).get("summary", {}) if pytest_results else {}
            self._update_aggregate_index(test_name, summary_data, str(filepath), pytest_summary)
            self._generate_index_html()
        except Exception as e:
            print(f"⚠️ Failed to update aggregate index: {e}")
        return str(filepath)

    def _generate_html_content(self, test_name: str,
                             execution_results: Dict[str, ProcessInfo],
                             summary_data: Dict[str, Any],
                             callback_results: Dict[str, Any] | None = None,
                             pytest_results: Dict[str, Any] | None = None) -> str:
        """Generate HTML report content"""

        # HTML模板
        html_template = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AimRT Test Report - {test_name}</title>
    <style>
        body {{
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
            line-height: 1.6;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            overflow: hidden;
        }}
        .header {{
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 30px;
            text-align: center;
        }}
        .header h1 {{
            margin: 0;
            font-size: 2.5em;
        }}
        .header p {{
            margin: 10px 0 0 0;
            opacity: 0.9;
        }}
        .summary {{
            padding: 30px;
            border-bottom: 1px solid #eee;
        }}
        .summary h2 {{
            color: #333;
            margin-top: 0;
        }}
        .stats-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
            margin-top: 20px;
        }}
        .stat-card {{
            background: #f8f9fa;
            padding: 20px;
            border-radius: 8px;
            border-left: 4px solid #007bff;
            text-align: center;
        }}
        .stat-value {{
            font-size: 2em;
            font-weight: bold;
            color: #007bff;
        }}
        .stat-label {{
            color: #666;
            margin-top: 5px;
        }}
        .processes {{
            padding: 30px;
        }}
        .callbacks {{
            padding: 0 30px 30px 30px;
        }}
        .callback-item {{
            border: 1px solid #ddd; margin: 10px 0; border-radius: 6px;
            padding: 10px; background: #fff;
        }}
        .callback-title {{ font-weight: bold; }}
        .ok {{ color: #155724; }}
        .fail {{ color: #721c24; }}
        .process-card {{
            border: 1px solid #ddd;
            border-radius: 8px;
            margin-bottom: 20px;
            overflow: hidden;
        }}
        .process-header {{
            background: #f8f9fa;
            padding: 15px;
            border-bottom: 1px solid #ddd;
            cursor: pointer;
        }}
        .process-header:hover {{
            background: #e9ecef;
        }}
        .process-title {{
            font-weight: bold;
            color: #333;
        }}
        .process-status {{
            display: inline-block;
            padding: 3px 8px;
            border-radius: 4px;
            font-size: 0.8em;
            font-weight: bold;
            margin-left: 10px;
        }}
        .status-completed {{ background: #d4edda; color: #155724; }}
        .status-failed {{ background: #f8d7da; color: #721c24; }}
        .status-timeout {{ background: #fff3cd; color: #856404; }}
        .status-killed {{ background: #d1ecf1; color: #0c5460; }}
        .process-details {{
            padding: 20px;
            display: none;
        }}
        .resource-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }}
        .resource-item {{
            background: #f8f9fa;
            padding: 15px;
            border-radius: 5px;
        }}
        .resource-title {{
            font-weight: bold;
            color: #495057;
            margin-bottom: 10px;
        }}
        .resource-value {{
            font-family: monospace;
            color: #666;
        }}
        .footer {{
            background: #f8f9fa;
            padding: 20px;
            text-align: center;
            color: #666;
            border-top: 1px solid #ddd;
        }}
        .toggle-icon {{
            float: right;
            transition: transform 0.3s;
        }}
        .expanded .toggle-icon {{
            transform: rotate(180deg);
        }}
        .children-section {{
            margin-top: 20px;
            padding: 15px;
            background: #f8f9fa;
            border-radius: 5px;
            border-left: 3px solid #28a745;
        }}
        .log-block {{
            max-height: 600px;
            overflow: auto;
            background: #0f172a;
            color: #e2e8f0;
            padding: 12px;
            border-radius: 6px;
            font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace;
            white-space: pre-wrap;
            word-break: break-word;
        }}

        /* pytest outcome styles */
        .outcome-pass {{ color: #155724; background: #d4edda; padding: 2px 6px; border-radius: 4px; display: inline-block; }}
        .outcome-fail {{ color: #721c24; background: #f8d7da; padding: 2px 6px; border-radius: 4px; display: inline-block; }}
        .outcome-skip {{ color: #856404; background: #fff3cd; padding: 2px 6px; border-radius: 4px; display: inline-block; }}
        .outcome-error {{ color: #0c5460; background: #d1ecf1; padding: 2px 6px; border-radius: 4px; display: inline-block; }}

        .children-title {{
            font-weight: bold;
            color: #28a745;
            margin-bottom: 10px;
        }}
        .child-item {{
            background: white;
            padding: 10px;
            margin: 5px 0;
            border-radius: 3px;
            border: 1px solid #dee2e6;
        }}
        .child-header {{
            font-weight: bold;
            color: #495057;
            margin-bottom: 5px;
        }}
        .child-details {{
            font-size: 0.9em;
            color: #666;
            font-family: monospace;
        }}
        .total-resources {{
            background: #e3f2fd;
            padding: 10px;
            border-radius: 5px;
            margin-top: 10px;
            border-left: 3px solid #2196f3;
        }}
        .total-resources-title {{
            font-weight: bold;
            color: #1976d2;
            margin-bottom: 5px;
        }}
    </style>
    <script>
        function toggleDetails(element) {{
            const details = element.nextElementSibling;
            const icon = element.querySelector('.toggle-icon');

            if (details.style.display === 'none' || !details.style.display) {{
                details.style.display = 'block';
                element.classList.add('expanded');
            }} else {{
                details.style.display = 'none';
                element.classList.remove('expanded');
            }}
        }}
    </script>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🚀 AimRT Test Report</h1>
            <p>{test_name} - {timestamp}</p>
        </div>

        <div class="summary">
            <h2>📊 Execution Summary</h2>
            <div class="stats-grid">
                <div class="stat-card">
                    <div class="stat-value">{total_processes}</div>
                    <div class="stat-label">Total processes</div>
                </div>
                <div class="stat-card">
                    <div class="stat-value">{completed}</div>
                    <div class="stat-label">Completed</div>
                </div>
                <div class="stat-card">
                    <div class="stat-value">{failed}</div>
                    <div class="stat-label">Failed</div>
                </div>
                <div class="stat-card">
                    <div class="stat-value">{success_rate:.1f}%</div>
                    <div class="stat-label">Success rate</div>
                </div>
                <div class="stat-card">
                    <div class="stat-value">{total_duration:.2f}s</div>
                    <div class="stat-label">Total duration</div>
                </div>
            </div>
        </div>

        <div class="processes">
            <h2>🔍 Process Details</h2>
            {process_cards}
        </div>

        <div class="callbacks">
            <h2>🧩 Callback Results</h2>
            {callback_cards}
        </div>

        <div class="footer">
            <p>Generated by AimRT test framework | {timestamp}</p>
        </div>
    </div>
</body>
</html>
        """

        # Build process cards
        process_cards_html = ""
        for script_path, process_info in execution_results.items():
            status_class = f"status-{process_info.status}"

            # Resource usage (without separate child details)
            resource_html = ""

            if process_info.monitor_data:
                from .resource_monitor import ResourceMonitor
                monitor = ResourceMonitor()
                resource_report = monitor.generate_report(process_info.monitor_data)

                resource_html = f"""
                <div class="resource-grid">
                    <div class="resource-item">
                        <div class="resource-title">💻 CPU Usage</div>
                        <div class="resource-value">
                            Avg: {resource_report['cpu']['avg_percent']:.1f}%<br>
                            Max: {resource_report['cpu']['max_percent']:.1f}%<br>
                            Final: {resource_report['cpu']['final_percent']:.1f}%
                        </div>
                    </div>
                    <div class="resource-item">
                        <div class="resource-title">🧠 Memory</div>
                        <div class="resource-value">
                            Avg: {resource_report['memory']['avg_rss_mb']:.1f}MB<br>
                            Max: {resource_report['memory']['max_rss_mb']:.1f}MB<br>
                            Percent: {resource_report['memory']['avg_percent']:.1f}%
                        </div>
                    </div>
                    <div class="resource-item">
                        <div class="resource-title">💾 Disk I/O</div>
                        <div class="resource-value">
                            Read: {resource_report['disk']['total_read_mb']:.2f}MB<br>
                            Write: {resource_report['disk']['total_write_mb']:.2f}MB<br>
                            Read speed: {resource_report['disk']['avg_read_mb_per_sec']:.2f}MB/s
                        </div>
                    </div>
                </div>
                """
            else:
                resource_html = "<p>⚠️ No resource monitoring data</p>"

            duration = (
                (process_info.end_time - process_info.start_time).total_seconds()
                if process_info.end_time else 0
            )

            # Escape logs safely to avoid breaking HTML
            stdout_html = escape(process_info.stdout or "")
            stderr_html = escape(process_info.stderr or "")

            process_card = f"""
            <div class="process-card">
                <div class="process-header" onclick="toggleDetails(this)">
                    <span class="process-title">{script_path}</span>
                    <span class="process-status {status_class}">{process_info.status}</span>
                    <span class="toggle-icon">▼</span>
                </div>
                <div class="process-details">
                    <p><strong>PID:</strong> {process_info.pid}</p>
                    <p><strong>Exit code:</strong> {process_info.exit_code}</p>
                    <p><strong>Duration:</strong> {duration:.2f}s</p>
                    <p><strong>Start time:</strong> {process_info.start_time.strftime('%Y-%m-%d %H:%M:%S')}</p>
                    <p><strong>End time:</strong> {process_info.end_time.strftime('%Y-%m-%d %H:%M:%S') if process_info.end_time else 'N/A'}</p>

                    <h4>📈 Resource Usage</h4>
                    {resource_html}

                    {("<h4>📤 Stdout</h4><pre class='log-block'>" + stdout_html + "</pre>") if stdout_html.strip() else ""}
                    {("<h4>📥 Stderr</h4><pre class='log-block'>" + stderr_html + "</pre>") if stderr_html.strip() else ""}
                </div>
            </div>
            """
            process_cards_html += process_card

        # 生成回调结果卡片
        callback_cards_html = ""
        if callback_results:
            # 先按回调名分组渲染
            for cb_name, results in callback_results.items():
                items_html = ""
                for r in results:
                    status_cls = "ok" if r.success else "fail"
                    msg = escape(r.message or "")
                    data = escape(str(getattr(r, 'data', {})))
                    warn = escape("; ".join(getattr(r, 'warnings', []) or []))
                    err = escape("; ".join(getattr(r, 'errors', []) or []))
                    items_html += f"<div class='callback-item'><div class='callback-title {status_cls}'>[{status_cls.upper()}] {msg}</div>" \
                                   f"<div>data: {data}</div>" \
                                   f"<div>warnings: {warn}</div>" \
                                   f"<div>errors: {err}</div></div>"
                callback_cards_html += f"<h3>{cb_name}</h3>" + items_html

        # Build pytest results section
        # Prepare pytest section (not yet displayed inline)
        _pytest_section_html = "<p>No pytest results</p>"
        if pytest_results and pytest_results.get("summary"):
            ps = pytest_results.get("summary", {})
            tests = pytest_results.get("tests", [])
            def map_outcome(o: str) -> tuple[str, str]:
                m = {
                    "passed": ("passed", "outcome-pass"),
                    "failed": ("failed", "outcome-fail"),
                    "skipped": ("skipped", "outcome-skip"),
                    "error": ("error", "outcome-error"),
                }
                return m.get((o or '').lower(), (escape(o or ''), ""))
            row_list = []
            for t in tests:
                label, cls = map_outcome(str(t.get('outcome','')))
                nodeid = escape(t.get('nodeid',''))
                dur = float(t.get('duration', 0) or 0)
                row_list.append(f"<tr><td>{nodeid}</td><td><span class=\"{cls}\">{label}</span></td><td>{dur:.3f}s</td></tr>")
            rows = "".join(row_list)
            _pytest_section_html = f"""
            <div class=\"resource-grid\">
                <div class=\"resource-item\">
                    <div class=\"resource-title\">Summary</div>
                    <div class=\"resource-value\">
                        Total: {ps.get('total',0)}, Passed: {ps.get('passed',0)}, Failed: {ps.get('failed',0)}, Skipped: {ps.get('skipped',0)}, Error: {ps.get('error',0)}, Success rate: {ps.get('success_rate',0):.1f}%
                    </div>
                </div>
                <div class=\"resource-item\" style=\"grid-column: 1 / -1;\">
                    <div class=\"resource-title\">Details</div>
                    <div class=\"resource-value\">
                        <table style=\"width:100%; border-collapse: collapse;\">
                            <thead>
                                <tr>
                                    <th style=\"text-align:left;border-bottom:1px solid #ddd;\">Case</th>
                                    <th style=\"text-align:left;border-bottom:1px solid #ddd;\">Outcome</th>
                                    <th style=\"text-align:left;border-bottom:1px solid #ddd;\">Duration</th>
                                </tr>
                            </thead>
                            <tbody>
                                {rows}
                            </tbody>
                        </table>
                    </div>
                </div>
            </div>
            """

        # Fill template
        summary = summary_data["summary"]
        # Note: pytest_section_html is currently not embedded; reserved for future use.
        return html_template.format(
            test_name=test_name,
            timestamp=datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            total_processes=summary["total_processes"],
            completed=summary["completed"],
            failed=summary["failed"],
            success_rate=summary["success_rate"],
            total_duration=summary["total_duration_seconds"],
            process_cards=process_cards_html,
            callback_cards=callback_cards_html
        )

    def generate_all_reports(self, test_name: str,
                           execution_results: Dict[str, ProcessInfo],
                           summary_data: Dict[str, Any],
                           callback_results: Dict[str, Any] | None = None,
                           pytest_results: Dict[str, Any] | None = None) -> Dict[str, str]:
        """
        Generate all report formats

        Returns:
            Dict[str, str]: mapping of format to output file path
        """
        reports = {}

        try:
            reports["json"] = self.generate_json_report(test_name, execution_results, summary_data, pytest_results, callback_results)
        except Exception as e:
            print(f"❌ Failed to generate JSON report: {e}")

        try:
            reports["html"] = self.generate_html_report(test_name, execution_results, summary_data, callback_results, pytest_results)
        except Exception as e:
            print(f"❌ Failed to generate HTML report: {e}")

        return reports

    def _generate_index_html(self):
        idx = self._load_index()
        entries = idx.get("entries", [])
        entries = list(reversed(entries))

        # Build multi-level tree: group by source YAML path
        from pathlib import Path as _P

        def _split_group_parts(yaml_path: str) -> list[str]:
            if not yaml_path:
                return []
            try:
                repo_root = _P(__file__).resolve().parents[2]
                rel = _P(yaml_path)
                try:
                    rel = rel.relative_to(repo_root)
                except Exception:
                    pass
                parts = list(rel.parts)
            except Exception:
                try:
                    parts = list(_P(yaml_path).parts)
                except Exception:
                    parts = []
            # Remove irrelevant prefixes (e.g. pytest_aimrt/generated)
            if 'pytest_aimrt' in parts:
                i = parts.index('pytest_aimrt')
                parts = parts[i+1:]
            if parts and parts[0] == 'generated':
                parts = parts[1:]
            # Keep directories only (strip filename)
            if parts and parts[-1].lower().endswith(('.yaml', '.yml')):
                parts = parts[:-1]
            return parts

        # Organize into nested dict; leaf nodes hold entry lists
        tree: dict = {}
        uncategorized: list = []
        for e in entries:
            meta = e.get('meta', {}) or {}
            yaml_path = meta.get('source_yaml_path', '') or ''
            parts = _split_group_parts(yaml_path)
            if parts:
                node = tree
                for p in parts:
                    node = node.setdefault(p, {})
                node.setdefault('_items', []).append(e)
            else:
                uncategorized.append(e)

        def _render_card(e) -> list[str]:
            s = e.get("summary", {})
            total = s.get("total_processes", 0)
            completed = s.get("completed", 0)
            failed = s.get("failed", 0)
            killed = s.get("killed", 0)
            timeout = s.get("timeout", 0)
            rate = s.get("success_rate", 0)
            # Color rule combines execution summary and pytest results:
            # - fail (red): execution failed>0 OR pytest failed/error>0
            # - warn (yellow): no items at all OR not 100% success (execution<100 or pytest has skips)
            # - ok (green): total>0 AND failed==0 AND execution rate==100% AND (pytest present -> all passed)
            ps = e.get('pytest_summary', {}) or {}
            p_total = ps.get('total', 0) or 0
            p_failed = ps.get('failed', 0) or 0
            p_error = ps.get('error', 0) or 0
            p_passed = ps.get('passed', 0) or 0
            meta = e.get('meta', {}) or {}
            cb_failures = bool(meta.get('callback_failures', False))
            cb_failed_count = int(meta.get('callback_failed_count', 0) or 0)

            # 显示成功率：若有回调失败，则强制显示为 0%
            display_rate = 0.0 if cb_failures else float(rate or 0)

            if cb_failures or (failed > 0) or (p_failed > 0) or (p_error > 0):
                badge_cls = "fail"
            elif (total == 0 and p_total == 0):
                badge_cls = "warn"
            elif (float(rate) < 100.0) or (p_total > 0 and p_passed < p_total):
                badge_cls = "warn"
            else:
                badge_cls = "ok"
            lines = []
            lines.append("<div class='card'>")
            lines.append(f"<div><a href='{Path(e['report_path']).name}' target='_blank'>{e['test_name']}</a>"
                         f" <span class='badge {badge_cls}'>成功率 {display_rate:.1f}%</span></div>")
            ps = e.get('pytest_summary', {}) or {}
            p_total = ps.get('total', 0)
            p_pass = ps.get('passed', 0)
            p_fail = ps.get('failed', 0)
            p_skip = ps.get('skipped', 0)
            p_err = ps.get('error', 0)
            p_rate = ps.get('success_rate', 0)
            py_html = ""
            if p_total:
                py_html = f" | Pytest: total {p_total} <span class='outp'>pass {p_pass}</span> <span class='outf'>fail {p_fail}</span> <span class='outs'>skip {p_skip}</span> <span class='oute'>error {p_err}</span> rate {p_rate:.1f}%"
            cb_html = f", 回调失败: {cb_failed_count}" if cb_failures else ""
            lines.append(f"<div class='meta'>时间: {e.get('timestamp','')}, 总进程: {total}, 完成: {completed}, 失败: {failed}, 超时: {timeout}, 强制终止: {killed}{cb_html}{py_html}</div>")
            lines.append("</div>")
            return lines

        def _render_tree(name: str, node: dict, depth: int = 0) -> list[str]:
            lines = []
            open_attr = " open" if depth <= 1 else ""
            lines.append(f"<details{open_attr}><summary>{name}</summary>")
            # 子目录
            for sub in sorted([k for k in node.keys() if k != '_items']):
                lines.extend(_render_tree(sub, node[sub], depth + 1))
            # 条目
            for e in node.get('_items', []):
                lines.extend(_render_card(e))
            lines.append("</details>")
            return lines

        # Generate HTML
        html = [
            "<!DOCTYPE html>",
            "<html lang='en'><head><meta charset='utf-8'>",
            "<meta name='viewport' content='width=device-width, initial-scale=1.0'>",
            "<title>AimRT Test Report Index</title>",
            "<style>body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:#f5f5f5;padding:20px} .container{max-width:1200px;margin:0 auto;background:#fff;border-radius:8px;box-shadow:0 2px 10px rgba(0,0,0,.1);overflow:hidden} .header{background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);color:#fff;padding:24px} .header h1{margin:0} .list{padding:20px} .card{border:1px solid #ddd;border-radius:8px;margin:12px 0;padding:12px;background:#fafafa} .meta{color:#666;font-size:12px} .badge{display:inline-block;padding:2px 6px;border-radius:4px;font-size:12px;margin-left:8px} .ok{background:#d4edda;color:#155724} .warn{background:#fff3cd;color:#856404} .fail{background:#f8d7da;color:#721c24} .outp{color:#155724;background:#d4edda;padding:2px 6px;border-radius:4px} .outf{color:#721c24;background:#f8d7da;padding:2px 6px;border-radius:4px} .oute{color:#0c5460;background:#d1ecf1;padding:2px 6px;border-radius:4px} .outs{color:#856404;background:#fff3cd;padding:2px 6px;border-radius:4px} details{margin:6px 0;padding:8px 12px;border:1px dashed #ddd;border-radius:6px;background:#fcfcfc} details summary{cursor:pointer;font-weight:600;color:#333} details details{margin-left:18px}</style>",
            "</head><body><div class='container'>",
            "<div class='header'><h1>🗂️ AimRT Test Report Index</h1>",
            f"<div class='meta'>Last updated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</div></div>",
            "<div class='list'>"
        ]

        # Render tree
        for root in sorted(tree.keys()):
            html.extend(_render_tree(root, tree[root], 0))

        # Uncategorized
        if uncategorized:
            html.append("<h3>Uncategorized</h3>")
            for e in uncategorized:
                html.extend(_render_card(e))

        html.append("</div></div></body></html>")
        self._index_html_path().write_text("\n".join(html), encoding='utf-8')

    def _index_json_path(self) -> Path:
        """返回聚合索引JSON文件路径（位于HTML目录）。"""
        return self.json_dir / "index.json"

    def _index_html_path(self) -> Path:
        """返回HTML入口页路径（位于HTML目录）。"""
        return self.html_dir / "index.html"

    def _load_index(self) -> Dict[str, Any]:
        """加载聚合索引JSON。"""
        path = self._index_json_path()
        if path.exists():
            try:
                return json.loads(path.read_text(encoding='utf-8'))
            except Exception:
                return {"entries": []}
        return {"entries": []}

    def _save_index(self, index_data: Dict[str, Any]):
        """保存聚合索引JSON。"""
        self._index_json_path().write_text(
            json.dumps(index_data, ensure_ascii=False, indent=2),
            encoding='utf-8'
        )

    def _update_aggregate_index(self, test_name: str, summary_data: Dict[str, Any], report_path: str, pytest_summary: Dict[str, Any] | None = None):
        """更新聚合索引，记录一次新的HTML报告。

        Args:
            test_name: 测试名称
            summary_data: 汇总数据
            report_path: HTML报告的绝对路径
        """
        idx = self._load_index()
        entries = idx.setdefault("entries", [])
        entry = {
            "test_name": test_name,
            "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            "summary": summary_data.get("summary", {}),
            "report_path": report_path,
            "pytest_summary": pytest_summary or {}
        }
        try:
            meta = summary_data.get("meta", {}) if isinstance(summary_data, dict) else {}
            if meta:
                entry["meta"] = meta
        except Exception:
            pass
        entries.append(entry)
        self._save_index(idx)