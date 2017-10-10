#!/usr/bin/env python


from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, MoveGroupAction
from moveit_msgs.srv import GetPlanningScene

import rospy
import moveit_commander
import actionlib

# workaround for core dump whenever exiting Python MoveIt script (https://github.com/ros-planning/moveit_commander/issues/15#issuecomment-34441531)
import atexit, os
atexit.register(lambda : os._exit(0))

# init rospy
rospy.init_node("publish_simple_scene")

# init moveit_commander
ac = actionlib.SimpleActionClient('move_group', MoveGroupAction)
ac.wait_for_server()
moveit_commander.MoveGroupCommander('arm')
scene_interface = moveit_commander.PlanningSceneInterface()
# get_planning_scene
try:
    rospy.wait_for_service('/get_planning_scene', timeout=20);
    get_planning_scene = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
except:
    get_planning_scene = None

# done
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.pose.position.x = 0.3
    pose.pose.position.y =-0.35
    pose.pose.position.z = 0.5
    pose.pose.orientation.w = 1.0
    scene_interface.add_box('simple_object_1', pose, (0.2,0.5,0.04))
    pose.pose.position.x = 0.3
    pose.pose.position.y =-0.12
    pose.pose.position.z = 0.72
    scene_interface.add_box('simple_object_2', pose, (0.2,0.04,0.4))
    pose.pose.position.x =-0.2
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.7
    scene_interface.add_box('simple_object_3', pose, (0.04,1.0,0.2))
    
    rate.sleep()

    print len(get_planning_scene(PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_NAMES)).scene.world.collision_objects)
    if get_planning_scene and len(get_planning_scene(PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_NAMES)).scene.world.collision_objects) >= 3:
        break

