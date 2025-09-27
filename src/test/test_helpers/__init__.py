# -*- coding: utf-8 -*-

"""
Alias package to expose `fixtures` and `core` located in the parent directory.
This allows importing `test_helpers.*` no matter where pytest is invoked under `src/test`.
"""

from typing import List
import pathlib

_parent_dir = pathlib.Path(__file__).resolve().parent.parent


try:
    __path__
except NameError:
    __path__: List[str] = []

if str(_parent_dir) not in __path__:
    __path__.append(str(_parent_dir))
