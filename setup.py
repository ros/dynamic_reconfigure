#!/usr/bin/env python

import os
from setuptools import setup
import sys
from xml.etree.ElementTree import ElementTree

try:
    root = ElementTree(None, 'stack.xml')
    version = root.findtext('version')
except Exception, e:
    print >> sys.stderr, 'Could not extract version from your stack.xml:\n%s' % e
    sys.exit(-1)

sys.path.insert(0, 'src')

PKG = 'dynamic_reconfigure'
gen = ['msg', 'srv']
packages = [PKG]
package_dir = {PKG: 'src/%s'%PKG}
if 'CATKIN_BINARY_DIR' in os.environ:
     build_d = os.environ['CATKIN_BINARY_DIR']
     for t in gen:
         p = os.path.join(build_d, 'gen', 'py', PKG, t)
         if os.path.isdir(p):
             # e.g. std_msgs.msg = build/gen/py/std_msgs/msg
             package_dir["%s.%s"%(PKG, t)] = p
             packages.append("%s.%s"%(PKG, t))

setup(name='dynamic_reconfigure',
      version= version,
      packages= packages, 
      package_dir = package_dir, 
      install_requires= ['ros_comm'],
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
