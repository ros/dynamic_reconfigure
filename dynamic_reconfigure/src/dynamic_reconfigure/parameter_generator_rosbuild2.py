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

# Author: Blaise Gassend

# Given a set of parameters, generates the messages, service types, and
# classes to allow runtime reconfiguration. Documentation of a node's
# parameters is a handy byproduct.

## @todo
# Need to check types of min max and default
# Need to put sane error on exceptions

import roslib; roslib.load_manifest("dynamic_reconfigure")
import roslib.packages
from string import Template
import os
import inspect
import string 
import sys

#LINEDEBUG="#line"
LINEDEBUG="//#line"

# Convenience names for types
str_t = "str"
bool_t = "bool"
int_t = "int"
double_t = "double"

class ParameterGenerator:
    minval = {
            'int' : -0x80000000, #'INT_MIN',
            'double' : '-std::numeric_limits<double>::infinity()',
            'str' : '',
            'bool' : False,
            }
            
    maxval = {
            'int' : 0x7FFFFFFF, #'INT_MAX',
            'double' : 'std::numeric_limits<double>::infinity()',
            'str' : '',
            'bool' : True,
            }
    
    defval = {
            'int' : 0,
            'double' : 0,
            'str' : '',
            'bool' : False,
            }
        
    def pytype(self, drtype):
        return { 'str':str, 'int':int, 'double':float, 'bool':bool }[drtype]


    def check_type(self, param, field):
        drtype = param['type']
        name = param['name']
        value = param[field]
        pytype = self.pytype(drtype)
        if pytype != type(value) and (pytype != float or type(value) != int):
            raise TypeError("'%s' has type %s, but %s is %s"%(name, drtype, field, repr(value)))
        param[field] = pytype(value)

    def fill_type(self, param):
        param['ctype'] = { 'str':'std::string', 'int':'int', 'double':'double', 'bool':'bool' }[param['type']]
        param['cconsttype'] = { 'str':'const char * const', 'int':'const int', 'double':'const double', 'bool':'const bool' }[param['type']]

    def check_type_fill_default(self, param, field, default):
        value = param[field]
        # If no value, use default.
        if value == None:
            param[field] = default
            return
        # Check that value type is compatible with type.
        self.check_type(param, field)
    
    def __init__(self):
        self.parameters = []
        self.constants = []
        self.dynconfpath = sys.argv[1]  # FIXME this is awful
        self.binary_dir = sys.argv[2]
        self.cpp_gen_dir = sys.argv[3]
        self.py_gen_dir = sys.argv[4]

    def const(self, name, type, value, descr):
        newconst = { 
                'name':name, 
                'type':type, 
                'value':value,
                'srcline' : inspect.currentframe().f_back.f_lineno,
                'srcfile' : inspect.getsourcefile(inspect.currentframe().f_back.f_code),
                'description' : descr
                }
        self.fill_type(newconst)
        self.check_type(newconst, 'value')
        self.constants.append(newconst)
        return newconst # So that we can assign the value easily.

    def enum(self, constants, description):
        if len(set(const['type'] for const in constants)) != 1:
            raise Exception("Inconsistent types in enum!")
        return repr({ 'enum' : constants, 'enum_description' : description }) 

    def add(self, name, paramtype, level, description, default = None, min = None, max = None, edit_method = ""):
        newparam = {
            'name' : name,
            'type' : paramtype,
            'default' : default,
            'level' : level,
            'description' : description,
            'min' : min,
            'max' : max,
            'srcline' : inspect.currentframe().f_back.f_lineno,
            'srcfile' : inspect.getsourcefile(inspect.currentframe().f_back.f_code),
            'edit_method' : edit_method,
        }
        if type == str_t and (max != None or min != None):
            raise Exception("Max or min specified for %s, which is of string type"%name)

        self.fill_type(newparam)
        self.check_type_fill_default(newparam, 'default', self.defval[paramtype])
        self.check_type_fill_default(newparam, 'max', self.maxval[paramtype])
        self.check_type_fill_default(newparam, 'min', self.minval[paramtype])
        self.parameters.append(newparam)

    def mkdirabs(self, path, second_attempt = False):
        if os.path.isdir(path):
            pass
        elif os.path.isfile(path):
            raise OSError("Error creating directory %s, a file with the same name exists" %path)
        elif second_attempt: # An exception occurred, but we still don't know why.
            raise
        else:
            os.makedirs(path)

    def generate(self, pkgname, nodename, name):
        try:
            if sys.modules['__main__']._DYNAMIC_RECONFIGURE_GENERATING_DEPENDENCIES:
                # Done this way because importing this module from gendeps
                # causes imports of dynamic_reconfigure.msg to fail from at
                # least some .cfg files. (Not sure why)
                return
        except:
            pass
        try:
            self.pkgname = pkgname
            self.name = name
            self.nodename = nodename
            self.msgname = name+"Config"
            #print '**************************************************************'
            #print '**************************************************************'
            print Template("Generating reconfiguration files for $name in $pkgname").\
                    substitute(name=self.name, pkgname = self.pkgname)
            #print '**************************************************************'
            #print '**************************************************************'
            self.generatecpp()
            self.generatedoc()
            self.generatewikidoc()
            self.generateusage()
            self.generatepy()
            #self.deleteobsolete()
        except Exception, e:
            print "Error building srv %s.srv"%name
            import traceback
            traceback.print_exc()
            exit(1)

    def generatewikidoc(self):
        self.mkdirabs(self.binary_dir, "docs")
        f = open(os.path.join(self.binary_dir, "docs", self.msgname+".wikidoc"), 'w')
        print >> f, \
"""# Autogenerated param section. Do not hand edit.
param {
group.0 {
name=Dynamically Reconfigurable Parameters
desc=See the [[dynamic_reconfigure]] package for details on dynamically reconfigurable parameters."""
        i=-1
        for param in self.parameters:
            i=i+1
            range = ""
            try:
              enum = eval(param['edit_method'])['enum']
              range = ", ".join(Template("$name ($value): $description").substitute(const) for const in enum)
              range = "Possible values are: " + range
            except:
              if param['type'] == int_t or param['type'] == double_t:
                  range = Template("Range: $min to $max").substitute(param)
            print >> f, Template(
"""$i.name= ~$name
$i.default= $default
$i.type= $type
$i.desc=$description $range"""
).substitute(param, range = range, i = i)
        print >> f,"}\n}\n# End of autogenerated section. You may edit below."
        f.close()

    def generateusage(self):
        self.mkdirabs("docs")
        f = open(os.path.join(self.binary_dir, "docs", self.msgname+"-usage.dox"), 'w')
        #print >> f, "/**"
        print >> f, "\\subsubsection usage Usage"
        print >> f, '\\verbatim'
        print >> f, Template('<node name="$nodename" pkg="$pkgname" type="$nodename">').\
                substitute(pkgname = self.pkgname, nodename = self.nodename)
        for param in self.parameters:
            print >> f, Template('  <param name="$name" type="$type" value="$default" />').substitute(param)
        print >> f, '</node>'
        print >> f, '\\endverbatim'
        print >> f
        #print >> f, "*/"
        f.close()
    
    def generatedoc(self):
        self.mkdirabs("docs")
        f = open(os.path.join(self.binary_dir, "docs", self.msgname+".dox"), 'w')
        #print >> f, "/**"
        print >> f, "\\subsubsection parameters ROS parameters"
        print >> f
        print >> f, "Reads and maintains the following parameters on the ROS server"
        print >> f
        for param in self.parameters:
            print >> f, Template("- \\b \"~$name\" : \\b [$type] $description min: $min, default: $default, max: $max").substitute(param)
        print >> f
        #print >> f, "*/"
        f.close()

    def generateusage(self):
        self.mkdirabs("docs")
        f = open(os.path.join(self.binary_dir, "docs", self.msgname+"-usage.dox"), 'w')
        #print >> f, "/**"
        print >> f, "\\subsubsection usage Usage"
        print >> f, '\\verbatim'
        print >> f, Template('<node name="$nodename" pkg="$pkgname" type="$nodename">').\
                substitute(pkgname = self.pkgname, nodename = self.nodename)
        for param in self.parameters:
            print >> f, Template('  <param name="$name" type="$type" value="$default" />').substitute(param)
        print >> f, '</node>'
        print >> f, '\\endverbatim'
        print >> f
        #print >> f, "*/"
        f.close()

    def crepr(self, param, val):
        type = param["type"]
        if type == 'str':
            return '"'+val+'"'
        if type in [ 'int', 'double']:
            return str(val)
        if  type == 'bool':
            return { True : 1, False : 0 }[val]
        raise TypeError(type)
#        if type == 'string':
#            return '"'+val+'"'
#        if 'uint' in type:
#            return str(val)+'ULL'
#        if 'int' in type:
#            return str(val)+'LL'
#        if 'time' in type:
#            return 'ros::Time('+str(val)+')'
#        if 'duration' in type:
#            return 'ros::Duration('+str(val)+')'
#        if  'float' in types:
#            return str(val)

    def appendline(self, list, text, param, value = None):
        if value == None:
            val = ""
        else:
            val = self.crepr(param, param[value])
        list.append(Template('${doline} $srcline "$srcfile"\n      '+text).safe_substitute(param, v=val, doline=LINEDEBUG, configname=self.name))
    
    def generatecpp(self):
        # Read the configuration manipulator template and insert line numbers and file name into template.
        templatefile = os.path.join(self.dynconfpath, "templates", "ConfigType.h")
        templatelines = []
        templatefilesafe = templatefile.replace('\\', '\\\\') # line directive does backslash expansion.
        curline = 1
        f = open(templatefile)
        for line in f:
            curline = curline + 1
            templatelines.append(Template(line).safe_substitute(linenum=curline,filename=templatefilesafe))
        f.close()
        template = ''.join(templatelines)
        
        # Write the configuration manipulator.
        self.mkdirabs(self.cpp_gen_dir)
        f = open(os.path.join(self.cpp_gen_dir, self.name+"Config.h"), 'w')
        paramdescr = []
        members = []
        constants = []
        for const in self.constants:
            self.appendline(constants, "${cconsttype} ${configname}_${name} = $v;", const, "value")
        for param in self.parameters:
            self.appendline(members, "${ctype} ${name};", param)
            self.appendline(paramdescr, "__min__.${name} = $v;", param, "min")
            self.appendline(paramdescr, "__max__.${name} = $v;", param, "max")
            self.appendline(paramdescr, "__default__.${name} = $v;", param, "default")
            self.appendline(paramdescr, 
                    "__param_descriptions__.push_back(${configname}Config::AbstractParamDescriptionConstPtr(new ${configname}Config::ParamDescription<${ctype}>(\"${name}\", \"${type}\", ${level}, "\
                    "\"${description}\", \"${edit_method}\", &${configname}Config::${name})));", param)
        paramdescr = string.join(paramdescr, '\n')
        members = string.join(members, '\n')
        constants = string.join(constants, '\n')
        f.write(Template(template).substitute(uname=self.name.upper(), 
            configname=self.name, pkgname = self.pkgname, paramdescr = paramdescr, 
            members = members, doline = LINEDEBUG, constants = constants))
        f.close()

#def deleteoneobsolete(self, file):
#         try:
#             os.unlink(file)
#         except OSError:
#             pass

#    def deleteobsolete(self): ### @todo remove this after the transition period.
#         self.deleteoneobsolete(os.path.join(self.pkgpath, "msg", self.msgname+".msg"))
#         self.deleteoneobsolete(os.path.join("msg", "cpp", self.pkgpath, "msg", self.msgname+".msg"))
#         self.deleteoneobsolete(os.path.join(self.pkgpath, "srv", "Get"+self.msgname+".srv"))
#         self.deleteoneobsolete(os.path.join("srv", "cpp", self.pkgpath, "srv", "Get"+self.msgname+".srv"))
#         self.deleteoneobsolete(os.path.join(self.pkgpath, "srv", "Set"+self.msgname+".srv"))
#         self.deleteoneobsolete(os.path.join("srv", "cpp", self.pkgpath, "srv", "Set"+self.msgname+".srv"))

#    def msgtype(self, type):
#        return { 'int' : 'int32', 'bool' : 'int8', 'str' : 'string', 'double' : 'float64' }[type]
#
#    def generatemsg(self):
#        self.mkdir("msg")
#        f = open(os.path.join(self.pkgpath, "msg", self.msgname+".msg"), 'w')
#        print >> f, "# This is an autogerenated file. Please do not edit."
#        print >> f, ""
#        for param in self.parameters:
#            print >> f, Template("$type $name # $description").substitute(param, type=self.msgtype(param['type']))
#        f.close()
#
#    def generategetsrv(self):
#        self.mkdir("srv")
#        f = open(os.path.join(self.pkgpath, "srv", "Get"+self.msgname+".srv"), 'w')
#        print >> f, "# This is an autogerenated file. Please do not edit."
#        print >> f, ""
#        print >> f, "---" 
#        print >> f, self.msgname, "config", "# Current configuration of node."
#        print >> f, self.msgname, "defaults", "# Minimum values where appropriate."
#        print >> f, self.msgname, "min", "# Minimum values where appropriate."
#        print >> f, self.msgname, "max", "# Maximum values where appropriate."
#        f.close()
#
#    def generatesetsrv(self):
#        self.mkdir("srv")
#        f = open(os.path.join(self.pkgpath, "srv", "Set"+self.msgname+".srv"), 'w')
#        print >> f, "# This is an autogerenated file. Please do not edit."
#        print >> f, self.msgname, "config", "# Requested node configuration."
#        print >> f, "---"        
#        print >> f, self.msgname, "config", "# What the node's configuration was actually set to."
#        f.close()
    
    def generatepy(self):
        # Read the configuration manipulator template and insert line numbers and file name into template.
        templatefile = os.path.join(self.dynconfpath, "templates", "ConfigType.py")
        templatelines = []
        f = open(templatefile)
        template = f.read()
        f.close()
        
        # Write the configuration manipulator.
        self.mkdirabs(os.path.join(self.py_gen_dir, "cfg"))
        f = open(os.path.join(self.py_gen_dir, "cfg", self.name+"Config.py"), 'w')
        f.write(Template(template).substitute(name = self.name, 
            pkgname = self.pkgname, pycfgdata = self.parameters))
        for const in self.constants:
            f.write(Template("${configname}_${name} = $v\n").
                    substitute(const, v = repr(const['value']), 
                        configname=self.name))
        f.close()

        f = open(os.path.join(self.py_gen_dir, "cfg", "__init__.py"), 'a')
        f.close()
