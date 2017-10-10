#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Tokyo Opensource Robotics Kyokai Association
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
#  * Neither the name of Tokyo Opensource Robotics Kyokai Association. nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
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
#
# Author: Isaac Isao Saito

#%Tag(FULLTEXT)%

import os
from subprocess import check_call

import actionlib_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_commander import MoveGroupCommander, conversions
import roslib
import rospy
import std_msgs.msg
from tf.transformations import quaternion_from_euler
#roslib.load_manifest("denso_pendant_publisher")
#roslib.load_manifest("actionlib_msgs")

rospy.init_node("test_vs060_moveit")

g_runnable = False
g_prev_status = None

arm = MoveGroupCommander("arm")
running_pub = rospy.Publisher("/irex_demo_running", std_msgs.msg.Bool);
#cancel_pub = rospy.Publisher("/arm_controller/follow_joint_trajectory/cancel", actionlib_msgs.msg.GoalID);
cancel_pub = rospy.Publisher("/move_group/cancel", actionlib_msgs.msg.GoalID);

# Get paths for files used later.
#roslib.packages.list_pkgs()
SCENE_FILE = roslib.packages.get_pkg_dir('vs060_moveit_config') + '/scene/irex_model.scene'
LOAD_SCENE_PROG = roslib.packages.find_node('vs060_moveit_config', 'publish_scene_from_text')[0]

print 'SCENE_FILE=', SCENE_FILE
print 'LOAD_SCENE_PROG=', LOAD_SCENE_PROG

def demo() :
    # load scene
    global g_runnable
    running_pub.publish(std_msgs.msg.Bool(g_runnable))
    check_call([LOAD_SCENE_PROG, SCENE_FILE])
    for p in [[ 0.35, -0.35, 0.4],
              [ 0.6,  0.0, 0.4],
              [ 0.35,  0.35, 0.4],
              [ 0.6,  0.0, 0.2],
              [ 0.4,  0.0, 0.8]]:
        running_pub.publish(std_msgs.msg.Bool(g_runnable))
        if g_runnable:
            print "set_pose_target(", p, ")"
            pose = PoseStamped(header = rospy.Header(stamp = rospy.Time.now(), frame_id = '/base_link'),
                               pose = Pose(position = Point(*p),
                                           orientation = Quaternion(*quaternion_from_euler(1.57, 0, 1.57, 'sxyz'))))

            arm.set_pose_target(pose)
            arm.go() or arm.go() or rospy.logerr("arm.go fails")
            rospy.sleep(1)
            if rospy.is_shutdown():
                return

if __name__ == "__main__":
    while not rospy.is_shutdown():
        demo()
#%EndTag(FULLTEXT)%
