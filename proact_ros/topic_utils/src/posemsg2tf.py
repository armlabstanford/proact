#!/usr/bin/env python
"""
Author: Shivani Guptasarma
Broadcast tf based on a pose message
Arguments: pose raw message topic, parent frame name, child frame name, invert or not. Use -i  to invert
"""

import rospy
from geometry_msgs.msg import PoseStamped
import sys
import tf
from pyquaternion import Quaternion
import numpy as np
import getopt

class posemsg2tf(object):
    def __init__(self, raw, parent, child, invert):
        rospy.sleep(1)
        self.rviz_frame_broadcaster=tf.TransformBroadcaster()
        self.raw = raw
        self.parent = parent
        self.child = child
        self.invert = invert
        print(invert)
        
    def posemsg2tf_callback(self, data):
        px= data.pose.position.x
        py= data.pose.position.y
        pz= data.pose.position.z
        qx= data.pose.orientation.x
        qy= data.pose.orientation.y
        qz= data.pose.orientation.z
        qw= data.pose.orientation.w

        if self.invert:
            pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
            quat = Quaternion(q0 = data.pose.orientation.w, q1 = data.pose.orientation.x, q2 = data.pose.orientation.y, q3 = data.pose.orientation.z)
            quat_new = quat.inverse
            pos_new = quat_new.rotate(-pos)
            px = pos_new[0]
            py = pos_new[1]
            pz = pos_new[2]
            qx = quat_new[1]
            qy = quat_new[2]
            qz = quat_new[3]
            qw = quat_new[0]
            print("inverted")

        self.rviz_frame_broadcaster.sendTransform((px,py,pz),
                         (qx,qy,qz,qw),
                         rospy.Time.now(),
                         self.child,
                         self.parent)

    def listener(self):
        rospy.Subscriber(self.raw, PoseStamped, self.posemsg2tf_callback)

if __name__ == '__main__':
    #create node
    rospy.init_node('posemsg2tf', anonymous=True) 
    #update rate
    # rate = rospy.Rate(10.0) # 10hz
    #create arm_publisher class
    argv = sys.argv[1:]
    try:
        opts, args = getopt.getopt(argv, "invert")
        print(opts, args)
    except:
        print("Error")

    invert = False

    for opt, arg in opts:
        if opt in ("-i", "--invert"):
            invert = True
        else:
            invert = False
 
    posemsg2tf_convertor=posemsg2tf(args[0], args[1], args[2], invert)
    # gazebo2tf_convertor.listener()
    # rospy.spin()
    # while not rospy.is_shutdown():
        # if not rospy.has_param('tracking_markers'):
    posemsg2tf_convertor.listener()
    rospy.spin()