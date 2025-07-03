#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# import sys
# import rospy
# import moveit_commander
# import geometry_msgs.msg

# def move_end_effector_z(group, delta_z):
#     # 获取当前末端位姿
#     pose = group.get_current_pose().pose

#     # 修改 Z 坐标
#     pose.position.z += delta_z

#     # 设置新的目标位姿
#     group.set_pose_target(pose)

#     # 执行运动
#     plan = group.go(wait=True)
#     group.stop()
#     group.clear_pose_targets()
#     return plan

# def main():
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node("dual_arm_safe_z_motion", anonymous=True)

#     # 获取左右臂的 MoveGroup 控制器
#     left_group = moveit_commander.MoveGroupCommander("left")
#     right_group = moveit_commander.MoveGroupCommander("right")

#     rospy.loginfo("Moving right arm end-effector UP by 10 cm...")
#     move_end_effector_z(right_group, -0.05)

#     rospy.sleep(1)

#     rospy.loginfo("Moving left arm end-effector DOWN by 10 cm...")
#     move_end_effector_z(left_group, -0.05)

# if __name__ == "__main__":
#     main()


import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import threading

def move_to_named_target(group, name):
    """Move the robot arm to a predefined named target position."""
    group.set_named_target(name)
    result = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return result

def move_end_effector_z(group, delta_z):
    """Move the end-effector along the Z-axis by a specified distance."""
    pose = group.get_current_pose().pose
    pose.position.z += delta_z
    group.set_pose_target(pose)
    result = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return result

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("dual_arm_safe_z_motion", anonymous=True)

    # Get MoveGroup controllers for both arms
    left_group = moveit_commander.MoveGroupCommander("left")
    right_group = moveit_commander.MoveGroupCommander("right")

    # Step 1: Move right arm to home
    rospy.loginfo("Moving right arm to home_right...")
    move_to_named_target(right_group, "home_right")
    rospy.sleep(1)

    # Step 2: Move left arm to home
    rospy.loginfo("Moving left arm to home_left...")
    move_to_named_target(left_group, "home_left")
    rospy.sleep(1)

    # Step 3: Move both arms simultaneously by 5 cm
    rospy.loginfo("Moving both arms by 5 cm simultaneously...")

    t1 = threading.Thread(target=move_end_effector_z, args=(left_group, 0.05))
    t2 = threading.Thread(target=move_end_effector_z, args=(right_group, 0.05))

    t1.start()
    t2.start()
    t1.join()
    t2.join()

    rospy.loginfo("Motion sequence complete.")

if __name__ == "__main__":
    main()
