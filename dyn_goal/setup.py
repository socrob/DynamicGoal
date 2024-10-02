#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['dyn_goal', 'dyn_goal_ros'],
 package_dir={'dyn_goal': 'common/src/dyn_goal', 'dyn_goal_ros': 'ros/src/dyn_goal_ros'}
)

setup(**d)
