#!/usr/bin/env python3
'''
Author: Shivani Guptasarma
Republish grasp events to a bool topic so Unity can listen and react
(ROS# unable to serialize custom messages)
'''

import rospy
import math
import std_msgs.msg
import gazebo_grasp_plugin_ros.msg

class grasp_pub:
    def __init__(self):
        self.pub = rospy.Publisher('/grasp_status', std_msgs.msg.Bool, queue_size=10)
        rospy.init_node('grasp_pub', anonymous=True)
        rospy.Subscriber("/grasp_event_republisher/grasp_events", gazebo_grasp_plugin_ros.msg.GazeboGraspEvent, self.callback)
        self.first_flush()
        rospy.spin()

    def first_flush(self):
        msgdata = std_msgs.msg.Bool()
        msgdata.data = True
        self.pub.publish(msgdata)
        msgdata.data = False
        self.pub.publish(msgdata)

    def callback(self, msg):
        msgdata = std_msgs.msg.Bool()
        msgdata.data = msg.attached
        self.pub.publish(msgdata)

if __name__ == '__main__':
    grasp_pub_obj = grasp_pub()