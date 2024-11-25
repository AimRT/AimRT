# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

from setuptools import find_packages, setup

package_name = 'aimrt_cli'

setup(
    name=package_name,
    app='aimrt_cli',
    version='0.0.5',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'PyYAML>=5.4.1',
        'Jinja2>=3.0.3',
        'pyinstaller>=6.1.0',
        'autopep8>=1.6.0',
    ],
    author=['Yu Xi', 'Yu Guanlin'],
    author_email=['yuxi@zhiyuan-robot.com', 'yuguanlin@agibot.com'],
    description='AimRT application python tools',
    license='',
    entry_points={
        'console_scripts': [
            'aimrt_cli = aimrt_cli.main:main',
        ],
    },
    package_data={
        'aimrt_cli': [
            'templates/**/*',
            'templates/**/**/*',
        ],
    },
    include_package_data=True,
)
