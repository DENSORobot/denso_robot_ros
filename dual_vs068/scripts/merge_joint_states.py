#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

merged_state = JointState()
merged_state.name = []
merged_state.position = []
merged_state.velocity = []
merged_state.effort = []

def update_joint_state(msg, source):
    global merged_state
    for i, name in enumerate(msg.name):
        if name in merged_state.name:
            idx = merged_state.name.index(name)
            merged_state.position[idx] = msg.position[i]
            merged_state.velocity[idx] = msg.velocity[i]
            merged_state.effort[idx] = msg.effort[i]
        else:
            merged_state.name.append(name)
            merged_state.position.append(msg.position[i])
            merged_state.velocity.append(msg.velocity[i])
            merged_state.effort.append(msg.effort[i])

def callback_l(msg):
    update_joint_state(msg, 'l')

def callback_r(msg):
    update_joint_state(msg, 'r')

if __name__ == '__main__':
    rospy.init_node('joint_state_merger_node')
    rospy.Subscriber('/vs068l/joint_states', JointState, callback_l)
    rospy.Subscriber('/vs068r/joint_states', JointState, callback_r)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        merged_state.header.stamp = rospy.Time.now()
        pub.publish(merged_state)
        rate.sleep()
