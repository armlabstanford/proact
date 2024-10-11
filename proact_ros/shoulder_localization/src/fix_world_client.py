#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from shoulder_localization.srv import *
import tf
import os

def fix_world_client():
    rospy.wait_for_service('fix_world')
    print("service found")
    try:
        fix_world = rospy.ServiceProxy('fix_world', fixWorld)
        print("service proxy created")
        resp = fix_world(True)
        x = resp.x
        y = resp.y
        z = resp.z 
        qx = resp.qx
        qy = resp.qy
        qz = resp.qz
        qw = resp.qw
        quaternion = (qx, qy, qz, qw)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        print(x, y, z, roll, pitch, yaw)
        os.system("rosrun tf static_transform_publisher " + str(x) + " " + str(y) + " " + str(z) + " " + str(yaw) + " " + str(pitch) + " " + str(roll) + " world hololens_world_test 100")
        # return True
        print("Requesting %s+%s"%(x, y))
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
    print("trying...")
    print("%s"%(fix_world_client()))