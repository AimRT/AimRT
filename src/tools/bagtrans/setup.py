# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

from setuptools import find_packages, setup

package_name = 'bagtrans'

setup(
    name=package_name,
    app='bagtrans',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    author='Yu Guanlin',
    author_email='yuguanlin@agibot.com',
    description='transfer aimrt bag file to ros2 bag file',
    license='',
    entry_points={
        'console_scripts': [
            'bagtrans = bagtrans.main:main',
        ],
    },
    include_package_data=True,
)
