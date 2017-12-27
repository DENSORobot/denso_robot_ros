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

__BRINGUP_TEXT = """<launch>
  <!-- the \"sim\" argument controls whether we connect to a Simulated or Real robot -->
  <!--  - if sim=false, a ip_address argument is required -->
  <arg name=\"sim\" default=\"true\" />
  <arg name=\"ip_address\" default=\"192.168.0.1\" />

  <!-- If you want to change send and recieve format of denso_robot_control, -->
  <!-- you can specify the send_format and recv_format parameters -->
  <arg name=\"send_format\" default=\"288\" />
  <arg name=\"recv_format\" default=\"292\" />

  <include file=\"$(find denso_robot_bringup)/launch/denso_robot_bringup.launch\">
    <arg name=\"robot_name\" value=\"{rob_name}\"/>
    <arg name=\"sim\" value=\"$(arg sim)\"/>
    <arg name=\"ip_address\" value=\"$(arg ip_address)\"/>
    <arg name=\"send_format\" value=\"$(arg send_format)\" />
    <arg name=\"recv_format\" value=\"$(arg recv_format)\" />
  </include>
</launch>"""

if __name__ == "__main__":
    args = sys.argv

    # Check num of arguments
    if len(args) < 2:
        print("Usage: install_robot_description \"path of description folder\"")
        sys.exit()

    path_desc = args[1]

    # Check path format
    m = re.search("([\w\d_]+)_description/?$", path_desc)
    if m == None:
        print("Invalid path format: it have to be *_description")
        sys.exit()

    rob_name = m.group(1)

    # Check path exists
    if not os.path.isdir(path_desc):
        print(path_desc + " does not exists")
        sys.exit()

    # Check config directory
    conf_dir = path_desc
    if conf_dir[-1] != "/":
        conf_dir += "/"
    conf_dir += rob_name + "_config"
    if not os.path.isdir(conf_dir):
        print(conf_dir + " does not exists")
        sys.exit()

    # Get package directory
    try:
        descs_pkg = roslib.packages.get_pkg_dir("denso_robot_descriptions")
        conf_pkg  = roslib.packages.get_pkg_dir("denso_robot_moveit_config")
        bringup_pkg = roslib.packages.get_pkg_dir("denso_robot_bringup")
    except roslib.packages.InvalidROSPkgException as e:
        print(e)
        sys.exit()

    if os.path.isdir(descs_pkg + "/" + rob_name + "_description"):
        print(rob_name + "_description is already in the denso_robot_descriptions package")
        sys.exit()

    if os.path.isdir(conf_pkg + "/config/" + rob_name + "_config"):
        print(rob_name + "_config is already in the denso_robot_moveit_config package")
        sys.exit()

    # Move config directory
    shutil.move(conf_dir, conf_pkg + "/config")

    # Move description directory
    shutil.move(path_desc, descs_pkg)

    # Make bringup launch file
    f = open(bringup_pkg + "/launch/" + rob_name + "_bringup.launch", "w")
    f.write(__BRINGUP_TEXT.replace("{rob_name}", rob_name))
    f.close()
