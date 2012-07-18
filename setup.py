#!/usr/bin/env python

from setuptools import setup

import sys
sys.path.insert(0, 'src')

setup(name='dynamic_reconfigure',
      version= '0.0.0',
      packages=['dynamic_reconfigure'],
      package_dir = {'':'src'},
      install_requires=['ros_comm'],
      author = "", 
      author_email = "",
      url = "",
      download_url = "", 
      keywords = [],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "",
      long_description = "",
      license = "BSD"
      )
