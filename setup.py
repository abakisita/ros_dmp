#!/usr/bin/env python

__version__ = '0.1'
__author__ = 'Abhishek Padalkar'

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['pydmps', 'ros_dmp'],
    package_dir={'pydmps': 'pydmps',
                 'ros_dmp': 'src/ros_dmp'}
)

setup(**d)
