# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Python client API for dynamic_reconfigure (L{DynamicReconfigureClient}) as well as 
example server implementation (L{DynamicReconfigureServer}).
"""

from __future__ import with_statement

import roslib; roslib.load_manifest('dynamic_reconfigure')
import rospy
import rosservice                  
import threading
import time
from dynamic_reconfigure import DynamicReconfigureParameterException
from dynamic_reconfigure.srv import Reconfigure as ReconfigureSrv
from dynamic_reconfigure.msg import Config as ConfigMsg
from dynamic_reconfigure.msg import ConfigDescription as ConfigDescrMsg
from dynamic_reconfigure.msg import IntParameter, BoolParameter, StrParameter, DoubleParameter, ParamDescription
from dynamic_reconfigure.encoding import *

class Client(object):
    """
    Python dynamic_reconfigure client API
    """
    def __init__(self, name, timeout=None, config_callback=None, description_callback=None):
        """
        Connect to dynamic_reconfigure server and return a client object
        
        @param name: name of the server to connect to (usually the node name)
        @type  name: str
        @param timeout: time to wait before giving up
        @type  timeout: float
        @param config_callback: callback for server parameter changes
        @param description_callback: internal use only as the API has not stabilized
        """
        self.name              = name
        self.config            = None
        self.param_description = None
        
        self._param_types = None

        self._cv = threading.Condition()

        self._config_callback      = config_callback
        self._description_callback = description_callback

        self._set_service      = self._get_service_proxy('set_parameters', timeout)       
        self._updates_sub      = self._get_subscriber('parameter_updates',      ConfigMsg,      self._updates_msg)
        self._descriptions_sub = self._get_subscriber('parameter_descriptions', ConfigDescrMsg, self._descriptions_msg)

    def get_configuration(self, timeout=None):
        """
        Return the latest received server configuration (wait to receive
        one if none have been received)

        @param timeout: time to wait before giving up
        @type  timeout: float
        @return: dictionary mapping parameter names to values or None if unable to retrieve config.
        @rtype: {str: value}
        """
        if timeout is None:
            with self._cv:
                while self.config is None:
                    if rospy.is_shutdown():
                        return None
                    self._cv.wait()
        else:
            start_time = time.time()      
            with self._cv:
                while self.config is None:
                    if rospy.is_shutdown():
                        return None
                    secs_left = timeout - (time.time() - start_time)
                    if secs_left <= 0.0:
                        break
                    self._cv.wait(secs_left)

        return self.config

    def get_parameter_descriptions(self, timeout=None):
        """
        UNSTABLE. Return a description of the parameters for the server.
        Do not use this method as the type that is returned may change.
        
        @param timeout: time to wait before giving up
        @type  timeout: float
        """
        if timeout is None:
            with self._cv:
                while self.param_description is None:
                    if rospy.is_shutdown():
                        return None
                    self._cv.wait()
        else:
            start_time = time.time()
            with self._cv:
                while self.param_description is None:
                    if rospy.is_shutdown():
                        return None
                    secs_left = timeout - (time.time() - start_time)
                    if secs_left <= 0.0:
                        break
                    self._cv.wait(secs_left)

        return self.param_description

    def update_configuration(self, changes):
        """
        Change the server's configuration

        @param changes: dictionary of key value pairs for the parameters that are changing
        @type  changes: {str: value}
        """
        # Retrieve the parameter descriptions
        if self.param_description is None:
            self.get_parameter_descriptions()

        # Cast the parameters to the appropriate types
        if self.param_description is not None:
            for name, value in changes.items()[:]:
                dest_type = self._param_types.get(name)
                if dest_type is None:
                    raise DynamicReconfigureParameterException('don\'t know type for parameter: %s' % name)
                
                changes[name] = dest_type(value)
        
        config = encode_config(changes)
        msg    = self._set_service(config).config
        resp   = decode_config(msg)

        return resp

    def close(self):
        """
        Close connections to the server
        """
        self._updates_sub.unregister()
        self._descriptions_sub.unregister()

    ## config_callback

    def get_config_callback(self):
        """
        Retrieve the config_callback
        """
        return self._config_callback

    def set_config_callback(self, value):
        """
        Set the config_callback
        """
        self._config_callback = value
        if self._config_callback is not None:
            self._config_callback(self.config)

    config_callback = property(get_config_callback, set_config_callback)

    ## description_callback        

    def get_description_callback(self):
        """
        Get the current description_callback
        """
        return self._config_callback

    def set_description_callback(self, value):
        """
        UNSTABLE. Set the description callback. Do not use as the type of the
        description callback may change.
        """
        self._description_callback = value
        if self._description_callback is not None:
            self._description_callback(self.param_description)

    description_callback = property(get_description_callback, set_description_callback)

    # Implementation

    def _get_service_proxy(self, suffix, timeout):
        service_name = rospy.resolve_name(self.name + '/' + suffix)
        rospy.wait_for_service(service_name, timeout)
        return rospy.ServiceProxy(service_name, ReconfigureSrv)
    
    def _get_subscriber(self, suffix, type, callback):
        topic_name = rospy.resolve_name(self.name + '/' + suffix)
        return rospy.Subscriber(topic_name, type, callback=callback)

    def _updates_msg(self, msg):
        self.config = decode_config(msg)
        
        with self._cv:
            self._cv.notifyAll()
        if self._config_callback is not None:
            self._config_callback(self.config)

    def _descriptions_msg(self, msg):
        self.param_description = decode_description(msg)

        # Build map from parameter name to type
        self._param_types = {}
        for p in self.param_description:
            n, t = p.get('name'), p.get('type')
            if n is not None and t is not None:
                self._param_types[n] = self._param_type_from_string(t)

        with self._cv:
            self._cv.notifyAll()
        if self._description_callback is not None:
            self._description_callback(self.param_description)

    def _param_type_from_string(self, type_str):
        if   type_str == 'int':    return int
        elif type_str == 'double': return float
        elif type_str == 'str':    return str
        elif type_str == 'bool':   return bool
        else:
            raise DynamicReconfigureParameterException('parameter has unknown type: %s. This is a bug in dynamic_reconfigure.' % type_str)
