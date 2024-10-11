#!/usr/bin/env python
"""
Author: Shivani Guptasarma
Publishes a link state to Gazebo based on tf
Arguments: parent tf frame, child tf frame, parent gazebo link, child gazebo link
"""

import rospy
from gazebo_msgs.srv import SetModelState, SetLinkState
from gazebo_msgs.msg import ModelState, LinkState
from geometry_msgs.msg import PoseStamped
import sys
import tf2_ros

#testing
import math

class Arm(object):
    def __init__(self, parent_tf, child_tf, parent_gazebo, child_gazebo):
        self.origin_pub_gazebo = rospy.Publisher('gazebo/set_link_state', LinkState, queue_size=10)
        self.new_link_state = LinkState()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.parent_tf = parent_tf
        self.child_tf = child_tf
        self.parent_gazebo = parent_gazebo
        self.child_gazebo = child_gazebo

    def publish_new_arm_position(self):
        try:
            self.origin_pub_gazebo.publish(self.new_link_state)
        except rospy.ROSInterruptException:
            pass

    def get_real_world_position(self):
        done = False
        while not done:
            try:
                trans = self.tfBuffer.lookup_transform(self.parent_tf, self.child_tf, rospy.Time())
                done = True
                # rospy.logwarn("found")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
                # rospy.logwarn("not found")
        self.new_link_state.reference_frame = 'world' #self.parent_gazebo
        self.new_link_state.link_name = 'mpl_right_arm::base_link' #self.child_gazebo
        self.new_link_state.pose.position.x = trans.transform.translation.x
        self.new_link_state.pose.position.y = trans.transform.translation.y
        self.new_link_state.pose.position.z = trans.transform.translation.z
        self.new_link_state.pose.orientation.x = trans.transform.rotation.x
        self.new_link_state.pose.orientation.y = trans.transform.rotation.y
        self.new_link_state.pose.orientation.z = trans.transform.rotation.z
        self.new_link_state.pose.orientation.w = trans.transform.rotation.w

    def refresh_arm_position(self):
        #get real world position in world frame
        self.get_real_world_position()
        #update info in gazebo
        self.publish_new_arm_position()


if __name__ == '__main__':
    #create node
    rospy.init_node('arm_placer', anonymous=True) #, log_level=rospy.WARN)

    #update rate
    rate = rospy.Rate(10.0) # 10hz

    #create arm_publisher class
    arm = Arm(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])

    while not rospy.is_shutdown():
        # if not rospy.has_param('tracking_markers'):
        arm.refresh_arm_position()
        rate.sleep()
