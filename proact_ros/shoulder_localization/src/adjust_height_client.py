#!/usr/bin/env python

from __future__ import print_function

# import sys
import rospy
from shoulder_localization.srv import *
import tf2_ros
# import os
import geometry_msgs.msg
# import tf_conversions

def adjust_height_client():
    rospy.wait_for_service('adjust_height')
    print("Service found")
    
    try:
        print("tried")
        adjust_height = rospy.ServiceProxy('adjust_height', adjustHeight)
        resp = adjust_height(True)
        x = 1.5
        y = 0
        z = -(1-(resp.z0-0.75)) #-(1-(resp.z0-0.49)) 
        qx = 0
        qy = 0
        qz = 0
        qw = 1
        # quaternion = (qx, qy, qz, qw)
        # euler = tf2_ros.transformations.euler_from_quaternion(quaternion)
        # roll = euler[0]
        # pitch = euler[1]
        # yaw = euler[2]
        # os.system("rosrun tf static_transform_publisher " + str(x) + " " + str(y) + " " + str(z) + " " + str(yaw) + " " + str(pitch) + " " + str(roll) + " tabletag_ros world 100")
        # return True
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "tabletag_ros"
        t.child_frame_id = "world"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        br.sendTransform(t)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    # if len(sys.argv) == 3:
    #     x = int(sys.argv[1])
    #     y = int(sys.argv[2])
    # else:
    #     print(usage())
    #     sys.exit(1)
    # print("Requesting %s+%s"%(x, y))
    print("%s"%(adjust_height_client()))