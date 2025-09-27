#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pytest results collection and export
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Dict, Any
from threading import Lock


@dataclass
class TestCaseResult:
    nodeid: str
    outcome: str  # passed/failed/skipped/error
    duration: float
    when: str  # setup/call/teardown (result phase)
    longrepr: str | None = None
    keywords: List[str] = field(default_factory=list)


class _ResultStore:
    def __init__(self) -> None:
        self._lock = Lock()
        self._results: List[TestCaseResult] = []

    def reset(self) -> None:
        with self._lock:
            self._results = []

    def record(self, result: TestCaseResult) -> None:
        with self._lock:
            # For the same nodeid, if there's an existing call-stage result, prefer error/failed
            if result.when == "call":
                # Remove previous call records for this nodeid
                self._results = [r for r in self._results if not (r.nodeid == result.nodeid and r.when == "call")]
                self._results.append(result)
            else:
                # If setup/teardown failed, record as error (higher priority)
                if result.outcome in ("failed", "error"):
                    # Remove any existing records for this nodeid and set as error
                    self._results = [r for r in self._results if r.nodeid != result.nodeid]
                    self._results.append(TestCaseResult(
                        nodeid=result.nodeid,
                        outcome="error",
                        duration=result.duration,
                        when=result.when,
                        longrepr=result.longrepr,
                        keywords=result.keywords,
                    ))

    def export(self) -> Dict[str, Any]:
        with self._lock:
            items = list(self._results)

        total = len(items)
        passed = sum(1 for r in items if r.outcome == "passed")
        failed = sum(1 for r in items if r.outcome == "failed")
        skipped = sum(1 for r in items if r.outcome == "skipped")
        error = sum(1 for r in items if r.outcome == "error")

        return {
            "summary": {
                "total": total,
                "passed": passed,
                "failed": failed,
                "skipped": skipped,
                "error": error,
                "success_rate": (passed / total * 100.0) if total > 0 else 0.0,
            },
            "tests": [
                {
                    "nodeid": r.nodeid,
                    "outcome": r.outcome,
                    "duration": r.duration,
                    "when": r.when,
                    "longrepr": r.longrepr,
                    "keywords": r.keywords,
                }
                for r in items
            ],
        }


_store = _ResultStore()


def reset_results() -> None:
    _store.reset()


def record_report(nodeid: str, outcome: str, duration: float, when: str, longrepr: str | None, keywords: List[str]):
    _store.record(TestCaseResult(
        nodeid=nodeid,
        outcome=outcome,
        duration=duration,
        when=when,
        longrepr=longrepr,
        keywords=keywords,
    ))


def export_results() -> Dict[str, Any]:
    return _store.export()
