#!/usr/bin/env python3
"""
Author: Shivani Guptasarma
Publishes a link state to Gazebo based on pose msg
Arguments: pose topic name, parent gazebo link, child gazebo link
"""

import rospy
from gazebo_msgs.srv import SetModelState, SetLinkState
from gazebo_msgs.msg import ModelState, LinkState
from geometry_msgs.msg import PoseStamped
import sys
import tf2_ros
import tf2_geometry_msgs


class Arm(object):
    def __init__(self, pose_topic, parent_gazebo, child_gazebo):
        self.origin_pub_gazebo = rospy.Publisher('gazebo/set_link_state', LinkState, queue_size=10)
        self.new_link_state = LinkState()
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.callback)
        self.parent_gazebo = parent_gazebo
        self.child_gazebo = child_gazebo

        # get optitrack world to real world transform NOW once
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.static_transform = None

        while self.static_transform is None:
            try:
                self.static_transform = self.tfBuffer.lookup_transform('world', 'optitrack_world', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

    def publish_new_arm_position(self):
        try:
            self.origin_pub_gazebo.publish(self.new_link_state)
        except rospy.ROSInterruptException:
            pass

    def callback(self, msg):
        self.new_link_state.reference_frame = 'world'
        self.new_link_state.link_name = 'mpl_right_arm::base_link'

        if self.static_transform is None:
            rospy.logerr("Failed to get static transform")
            return

        try:
            transformed_pose = tf2_geometry_msgs.do_transform_pose(msg, self.static_transform)
            # print(msg)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to transform pose")
        
        self.new_link_state.pose.position.x = transformed_pose.pose.position.x
        self.new_link_state.pose.position.y = transformed_pose.pose.position.y
        self.new_link_state.pose.position.z = transformed_pose.pose.position.z
        self.new_link_state.pose.orientation.x = transformed_pose.pose.orientation.x
        self.new_link_state.pose.orientation.y = transformed_pose.pose.orientation.y
        self.new_link_state.pose.orientation.z = transformed_pose.pose.orientation.z
        self.new_link_state.pose.orientation.w = transformed_pose.pose.orientation.w
        self.publish_new_arm_position()

if __name__ == '__main__':
    try:
        pose_topic = sys.argv[1]
        parent_gazebo = sys.argv[2]
        child_gazebo = sys.argv[3]

        rospy.init_node('tf2gazebo', anonymous=True)

        arm_obj = Arm(pose_topic, 'world', 'mpl_right_arm::base_link')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass