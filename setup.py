#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['dynamic_reconfigure']
d['package_dir'] = {'': 'src'}
d['requires'] = ['roslib', 'rospy', 'rosservice']

setup(**d)
