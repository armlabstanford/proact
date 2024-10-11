#!/usr/bin/env python

from __future__ import print_function
import rospy
import tf2_ros
import os
import geometry_msgs.msg
import tf_conversions

def adjust_height_node():
    rospy.init_node('adjust_height_node')
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
    z0= trans.transform.translation.y
    x = 1.5
    y = 0
    z = -(1-(z0-0.43)) #-(1-(z0-0.50))
    qx = 0
    qy = 0
    qz = 0
    qw = 1
    quaternion = (qx, qy, qz, qw)
    euler = tf_conversions.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    os.system("rosrun tf static_transform_publisher " + str(x) + " " + str(y) + " " + str(z) + " " + str(yaw) + " " + str(pitch) + " " + str(roll) + " tabletag_ros world 100")
    # return True
    # br = tf2_ros.TransformBroadcaster()
    # t = geometry_msgs.msg.TransformStamped()

    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "tabletag_ros"
    # t.child_frame_id = "world"
    # t.transform.translation.x = x
    # t.transform.translation.y = y
    # t.transform.translation.z = z
    # # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    # t.transform.rotation.x = qx
    # t.transform.rotation.y = qy
    # t.transform.rotation.z = qz
    # t.transform.rotation.w = qw

    # br.sendTransform(t)
    
    rospy.spin()

if __name__ == "__main__":
    adjust_height_node()