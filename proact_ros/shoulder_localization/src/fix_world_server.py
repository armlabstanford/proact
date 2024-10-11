#!/usr/bin/env python3

from __future__ import print_function
import rospy
from shoulder_localization.srv import fixWorld, fixWorldResponse
import tf2_ros

def fix_world(req):
    # If setting up publisher
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # Get current hololens_world_mocap
    done = False
    while not done:
        try:
            trans = tfBuffer.lookup_transform('world', 'hololens_world_mocap', rospy.Time())
            done = True
            # rospy.logwarn("found")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
    # quaternion = (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
    # euler = tf.transformations.euler_from_quaternion(quaternion)
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]
    # os.system("rosrun tf static_transform_publisher " + str(trans.transform.translation.x) + " " + str(trans.transform.translation.y) + " " + str(trans.transform.translation.z) + " " + str(yaw) + " " + str(pitch) + " " + str(roll) + " world hololens_world_test 100")
    return fixWorldResponse(x= trans.transform.translation.x, y=trans.transform.translation.y, z=trans.transform.translation.z, qx=trans.transform.rotation.x, qy=trans.transform.rotation.y, qz=trans.transform.rotation.z, qw=trans.transform.rotation.w)

def fix_world_server():
    rospy.init_node('fix_world_server')
    s = rospy.Service('fix_world', fixWorld, fix_world)
    rospy.spin()

if __name__ == "__main__":
    fix_world_server()