#!/usr/bin/env python3
"""
Author: Shivani Guptasarma
Provided the pose of the Hololens from motion capture and the pose of its camera from Unity app,
with pre-calibrated transform between motion capture frame and camera frame, 
finds Hololens world with respect to Gazebo world so that Gazebo objects can be localized in physical space. 

- Adds hololens_world to tf tree
- Replaces hololens_world.cpp
- If not working (if markers moved etc), use longer procedure with Apriltag, then run 
`rosrun tf tf_echo hololens_camera_frame <name of Hololens frame from motion capture>`
"""
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
import sys
import tf
import tf2_ros
import math
import numpy as np
from pyquaternion import Quaternion

# arguments: pose raw message topic, parent frame name, child frame name

class world_setter(object):
    def __init__(self):
        rospy.sleep(1)
        self.rviz_frame_broadcaster=tf.TransformBroadcaster()
        # transform from mocap frame to head frame (transform that moves a coordinate frame from head to mocap frame, or a vector in the opposite direction;
        # this is obtained by rosrun tf tf_echo holohead <hololens_mocap>)
        # **************************
        # note that q0 is first here, not last! In the terminal output, it is last

        self.pos_m2c = np.array([0.027, 0.017, 0.083]) #[0.020, 0.000, 0.072]
        temp = np.array([0.448, 0.453, 0.554, 0.536]) #[0.521, 0.369, 0.489, 0.594]
        self.quat_m2c = Quaternion(q0 = temp[3], q1 = temp[0], q2 = temp[1], q3 = temp[2])

        quat_c2m = self.quat_m2c.inverse
        pos_c2m = quat_c2m.rotate(-self.pos_m2c)

        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "HololensSG"
        static_transformStamped.child_frame_id = "hololens_unity_camera"

        static_transformStamped.transform.translation.x = self.pos_m2c[0]
        static_transformStamped.transform.translation.y = self.pos_m2c[1]
        static_transformStamped.transform.translation.z = self.pos_m2c[2]
        static_transformStamped.transform.rotation.x = self.quat_m2c[0]
        static_transformStamped.transform.rotation.y = self.quat_m2c[1]
        static_transformStamped.transform.rotation.z = self.quat_m2c[2]
        static_transformStamped.transform.rotation.w = self.quat_m2c[3]

        broadcaster.sendTransform(static_transformStamped)
        
    def world_setter_callback(self, data):
        # receive camera to world transform from hololens 
        pos_c2w = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        quat_c2w = Quaternion(q0 = data.pose.orientation.w, q1 = data.pose.orientation.x, q2 = data.pose.orientation.y, q3 = data.pose.orientation.z)

        quat_m2w = quat_c2w*self.quat_m2c
        pos_m2w = quat_c2w.rotate(self.pos_m2c) + pos_c2w

        quat_w2m = quat_m2w.inverse
        pos_w2m = quat_w2m.rotate(-pos_m2w)

        self.rviz_frame_broadcaster.sendTransform((pos_w2m[0],pos_w2m[1],pos_w2m[2]),
                         (quat_w2m[1], quat_w2m[2], quat_w2m[3], quat_w2m[0]),
                         rospy.Time.now(),
                         "hololens_world_mocap",
                         "HololensSG")
        
    def listener(self):
        # rospy.Subscriber("/holocam/webcam_to_world", PoseStamped, self.world_setter_callback)
        rospy.Subscriber("/hololens/head", PoseStamped, self.world_setter_callback)

if __name__ == '__main__':
    #create node
    rospy.init_node('hololens_world_establish')
    world_setter_object=world_setter()
    world_setter_object.listener()
    rospy.spin()
