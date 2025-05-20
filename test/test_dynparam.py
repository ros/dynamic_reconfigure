#!/usr/bin/env python

import unittest
import rostest
import rospy
import os
import subprocess
import tempfile


class TestDynparam(unittest.TestCase):

    def test_dynparam(self):
        unsafe_yaml_file = os.path.join(os.path.abspath(os.path.join(os.path.dirname(__file__))), 'test_unsafe_yaml.yaml')
        retcode_unsafe_yaml = 0

        try:
            # run dynparam load with insecure yaml
            output = subprocess.check_output(['rosrun dynamic_reconfigure dynparam load ref_server ' + unsafe_yaml_file], stderr=subprocess.STDOUT, shell=True, executable="/bin/bash")
        except subprocess.CalledProcessError as e:
            self.assertIn('could not determine a constructor for the tag'.encode(), e.output) # check the right error is being thrown
            retcode_unsafe_yaml = e.returncode

        # check the test failed
        self.assertNotEqual(retcode_unsafe_yaml, 0)

    def test_dynparam_dump_and_load(self):
        # Dump parameters
        yaml_file = tempfile.NamedTemporaryFile()
        subprocess.check_call(['rosrun', 'dynamic_reconfigure', 'dynparam', 'dump', 'ref_server', yaml_file.name])
        # Load parameters
        subprocess.check_call(['rosrun', 'dynamic_reconfigure', 'dynparam', 'load', 'ref_server', yaml_file.name])


if __name__ == '__main__':
    rospy.init_node('dynparam_test')
    rospy.sleep(3.0)
    rostest.rosrun('dynamic_reconfigure', 'test_dynparam', TestDynparam)
