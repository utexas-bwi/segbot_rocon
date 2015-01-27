#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['segbot_concert_services'],
    package_dir={'': 'src'},
    scripts={'scripts/multi_robot_patroller', 'scripts/segbot_requestor'},
    )

setup(**d)
