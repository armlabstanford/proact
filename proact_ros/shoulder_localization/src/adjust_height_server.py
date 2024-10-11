#!/usr/bin/env python

from __future__ import print_function
import rospy
from shoulder_localization.srv import adjustHeight, adjustHeightResponse
import tf2_ros

def adjust_height(req):
    # If setting up publisher
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # Get current hololens_world_mocap
    done = False
    while not done:
        try:
            trans = tfBuffer.lookup_transform('Clipboard', 'Shoulderpad', rospy.Time())
            done = True
            # rospy.logwarn("found")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
    return adjustHeightResponse(z0= trans.transform.translation.y)

def adjust_height_server():
    rospy.init_node('adjust_height_server')
    s = rospy.Service('adjust_height', adjustHeight, adjust_height)
    rospy.spin()

if __name__ == "__main__":
    adjust_height_server()