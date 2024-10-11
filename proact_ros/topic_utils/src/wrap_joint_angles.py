#!/usr/bin/env python
"""
Author: Shivani Guptasarma
Wrap joint angles to (-pi, pi) and republish
"""

import rospy
import math
import sensor_msgs.msg

def callback(msg):
    pub = rospy.Publisher('/mpl_right_arm/wrapped_joint_states', sensor_msgs.msg.JointState, queue_size=10)
    data = sensor_msgs.msg.JointState()
    data.header = msg.header
    data.name = msg.name
    data.velocity = msg.velocity
    data.effort = msg.effort
    pos_list = list(msg.position)
    for i in range(len(pos_list)):
        if (pos_list[i] > math.pi):
            while (pos_list[i] > math.pi):
                pos_list[i] = pos_list[i] - 2*math.pi
        elif (pos_list[i] < -math.pi):
            while (pos_list[i] < -math.pi):
                pos_list[i] = pos_list[i] + 2*math.pi
                # floats will never be exactly == math.pi
    data.position = tuple(pos_list)
    pub.publish(data)
    
def listener():

    rospy.init_node('robot_state_wrapper', anonymous=True)
    rospy.Subscriber("/mpl_right_arm/joint_states", sensor_msgs.msg.JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()