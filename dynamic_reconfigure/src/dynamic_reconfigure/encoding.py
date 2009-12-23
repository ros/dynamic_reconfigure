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

import roslib; roslib.load_manifest('dynamic_reconfigure')
import rospy

from dynamic_reconfigure.msg import Config as ConfigMsg
from dynamic_reconfigure.msg import ConfigDescription as ConfigDescrMsg
from dynamic_reconfigure.msg import IntParameter, BoolParameter, StrParameter, DoubleParameter, ParamDescription

def encode_description(descr):
    msg = ConfigDescrMsg()
    msg.max = encode_config(descr.max)
    msg.min = encode_config(descr.min)
    msg.dflt = encode_config(descr.defaults)
    for param in descr.config_description:
        msg.parameters.append(ParamDescription(param['name'], param['type'], param['level'], param['description'], param['edit_method']))
    return msg

def encode_config(config):
    msg = ConfigMsg()
    for k, v in config.items():
        ## @todo add more checks here?
        if   type(v) == int:   msg.ints.append(IntParameter(k, v))
        elif type(v) == bool:  msg.bools.append(BoolParameter(k, v))
        elif type(v) == str:   msg.strs.append(StrParameter(k, v))
        elif type(v) == float: msg.doubles.append(DoubleParameter(k, v))
    return msg

def decode_description(msg):
    descr = []
    mins = decode_config(msg.min)
    maxes = decode_config(msg.max)
    defaults = decode_config(msg.dflt)
    for param in msg.parameters:
        name = param.name
        descr.append({
            'name': name,
            'min': mins[name],
            'max': maxes[name],
            'default': defaults[name],
            'type': param.type,
            'description': param.description,
            'edit_method': param.edit_method,
            })
    return descr

def decode_config(msg):
    return dict([(kv.name, kv.value) for kv in msg.bools + msg.ints + msg.strs + msg.doubles])
