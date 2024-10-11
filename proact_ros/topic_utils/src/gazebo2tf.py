#!/usr/bin/env python
"""
Author: Shivani Guptasarma
Broadcast tf based on gazebo state
Arguments: model name, parent model name in gazebo, parent tf name
"""

import rospy
from gazebo_msgs.srv import GetModelState, GetLinkState
from gazebo_msgs.msg import ModelState, LinkState
from geometry_msgs.msg import PoseStamped
import sys
import tf

class gazebo2tf(object):
    def __init__(self, model, parent, parent_tf):
        rospy.sleep(1)
        self.rviz_frame_broadcaster=tf.TransformBroadcaster()
        self.model_state=ModelState()
        self.model_pose=PoseStamped()
        self.parent = parent
        self.parent_tf = parent_tf
        self.model = model
        
    def gazebo2tf_callback(self):
        px= self.model_state.pose.position.x
        py= self.model_state.pose.position.y
        pz= self.model_state.pose.position.z
        qx= self.model_state.pose.orientation.x
        qy= self.model_state.pose.orientation.y
        qz= self.model_state.pose.orientation.z
        qw= self.model_state.pose.orientation.w

        print("callback")

        self.rviz_frame_broadcaster.sendTransform((px,py,pz),
                         (qx,qy,qz,qw),
                         rospy.Time.now(),
                         self.model,
                         self.parent_tf)

    def listener(self):
        rospy.wait_for_service('gazebo/get_link_state')
        try:
            sub_gazebo = rospy.ServiceProxy('gazebo/get_model_state', GetModelState) 
            self.model_state = sub_gazebo(self.model, self.parent)
            self.gazebo2tf_callback()
            return 
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


if __name__ == '__main__':
    rospy.init_node('gazebo2tf', anonymous=True) 
    rate = rospy.Rate(10.0) # 10hz
    gazebo2tf_convertor=gazebo2tf(sys.argv[1], sys.argv[2], sys.argv[3])
    while not rospy.is_shutdown():
        gazebo2tf_convertor.listener()
        rate.sleep()