#!/usr/bin/env python  
"""
Author: Shivani Guptasarma
Broadcast a pose stamped message based on tf 
Arguments: message topic name, parent tf frame, child tf frame
"""

import rospy
import sys
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_to_unity', anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)


    pose_pub = rospy.Publisher(sys.argv[1], geometry_msgs.msg.PoseStamped, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(sys.argv[2], sys.argv[3], rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = geometry_msgs.msg.PoseStamped()

        msg.pose.position.x = trans.transform.translation.x
        msg.pose.position.y = trans.transform.translation.y
        msg.pose.position.z = trans.transform.translation.z
        msg.pose.orientation.x = trans.transform.rotation.x
        msg.pose.orientation.y = trans.transform.rotation.y
        msg.pose.orientation.z = trans.transform.rotation.z
        msg.pose.orientation.w = trans.transform.rotation.w
        
        pose_pub.publish(msg)

        rate.sleep()