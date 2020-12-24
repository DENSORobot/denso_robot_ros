#!/usr/bin/env python

'''
 Software License Agreement (MIT License)

 @copyright Copyright (c) 2017 DENSO WAVE INCORPORATED

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
'''

import os
import re
import sys
import rospy
import shutil
import roslib.packages
import xml.etree.ElementTree as ET

if __name__ == "__main__":
    args = sys.argv

    # Check num of arguments
    if len(args) < 3:
        print("Usage: install_robot_description \"robot name\" \"path of joint_limits.yaml\"")
        sys.exit()

    rob_name = args[1]
    path_limits = args[2]

    # Check path exists
    if not os.path.isfile(path_limits):
        print(path_limits + " does not exists")
        sys.exit()

    # Get package directory
    try:
        descs_pkg = roslib.packages.get_pkg_dir("denso_robot_descriptions")
        conf_pkg  = roslib.packages.get_pkg_dir("denso_robot_moveit_config")
    except roslib.packages.InvalidROSPkgException as e:
        print(e)
        sys.exit()

    path_desc = descs_pkg + "/" + rob_name + "_description"
    if not os.path.isdir(path_desc):
        print(rob_name + "_description is not exists")
        sys.exit()

    path_urdf = path_desc + "/" + rob_name + ".urdf"
    if not os.path.isfile(path_urdf):
        print(rob_name + ".urdf is not exists")
        sys.exit()

    path_conf = conf_pkg + "/config/" + rob_name + "_config"
    if not os.path.isdir(path_conf):
        print(rob_name + "_config is not exists")
        sys.exit()

    et_urdf = ET.parse(path_urdf)
    elem_joints = et_urdf.getroot().findall("joint")

    r_joint = re.compile("(joint_\d+):")
    r_has_vel_limit = re.compile("has_velocity_limits: (\w+)")
    r_max_vel = re.compile("max_velocity: ([\d\.]+)")

    cur_elem = None
    new_joint = False

    f_limits = open(path_limits, "r+")

    # Insert space after colon.
    data_lines = f_limits.read()
    index = data_lines.find(": ")
    if index == -1:
        new_data = re.sub(r':([\w\.]+)', r': \1', data_lines)
        f_limits.seek(0)
        f_limits.write(new_data)
        f_limits.truncate()

    f_limits.seek(0)
    line = f_limits.readline()
    while line:
        m = r_joint.search(line)
        if not m == None:
            for elem in elem_joints:
                if elem.attrib["name"] == m.group(1):
                    cur_elem = elem
                    new_joint = True
                    break

        if new_joint:
            m = r_has_vel_limit.search(line)
            if (not m == None) and (m.group(1) == "false"):
                cur_elem.find("limit").attrib["velocity"] = "1"
                new_joint = False

        if new_joint:
            m = r_max_vel.search(line)
            if not m == None:
                cur_elem.find("limit").attrib["velocity"] = m.group(1)
                new_joint = False

        line = f_limits.readline()
    f_limits.close()

    # Update urdf file
    et_urdf.write(path_urdf)

    # Move joint_limits.yaml
    shutil.copy(path_limits, path_conf)
    os.remove(path_limits)
