# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

from setuptools import setup
from setuptools.dist import Distribution


class BinaryDistribution(Distribution):
    def has_ext_modules(self):
        return True


with open("VERSION", "r") as f:
    version = f.read().strip()

setup(
    version=version,
    distclass=BinaryDistribution,
)
