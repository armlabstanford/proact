#!/usr/bin/env python3
"""
Author: Shivani Guptasarma
Average pose over a window and republish
Arguments:  pose raw message topic, new message topic, window_size
"""

import rospy
import copy
from geometry_msgs.msg import PoseStamped
import sys
import numpy as np
import scipy
from scipy.spatial.transform import Rotation as R

class pose_filter(object):
    def __init__(self, raw, new, window_size):
        rospy.sleep(1)
        self.pose_sub     = rospy.Subscriber(raw, PoseStamped, self.pose_filter_callback)
        self.filtered_pub = rospy.Publisher(new, PoseStamped, queue_size=1)
        self.window_size = int(window_size)
        self.cache = []
        self.prev_pose = np.array([0,0,0,0,0,0,1])

    def pose_filter_callback(self, data):

        rot = R.from_quat([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        rotvec = rot.as_rotvec()

        # add new data to end of window
        entry = [data.pose.position.x,data.pose.position.y,data.pose.position.z, rotvec[0], rotvec[1], rotvec[2]]
        self.cache.append(entry)

        # pop old ones
        if len(self.cache) > self.window_size:
            self.cache.pop(0)

        # find mean position
        averaged_x = np.mean(np.array([i[0] for i in self.cache]))
        averaged_y = np.mean(np.array([i[1] for i in self.cache]))
        averaged_z = np.mean(np.array([i[2] for i in self.cache]))

        averaged_rotvec_x = np.mean(np.array([i[3] for i in self.cache]))
        averaged_rotvec_y = np.mean(np.array([i[4] for i in self.cache]))
        averaged_rotvec_z = np.mean(np.array([i[5] for i in self.cache]))

        averaged_rotvec = np.array([averaged_rotvec_x, averaged_rotvec_y, averaged_rotvec_z])

        # # mean orientation 
        rot_avg = R.from_rotvec(averaged_rotvec)
        quat_avg = rot_avg.as_quat()

        # # prepare message for publishing
        filtered = copy.deepcopy(data)
        filtered.header.stamp = rospy.Time.now()
        filtered.pose.position.x = averaged_x
        filtered.pose.position.y = averaged_y
        filtered.pose.position.z = averaged_z

        filtered.pose.orientation.x = quat_avg[0]
        filtered.pose.orientation.y = quat_avg[1]
        filtered.pose.orientation.z = quat_avg[2]
        filtered.pose.orientation.w = quat_avg[3]

        prev_x = self.prev_pose[0]
        prev_y = self.prev_pose[1]
        prev_z = self.prev_pose[2]
        prev_qx = self.prev_pose[3]
        prev_qy = self.prev_pose[4]
        prev_qz = self.prev_pose[5]
        prev_qw = self.prev_pose[6]

        error = scipy.linalg.norm(np.array([averaged_x-prev_x, averaged_y-prev_y, averaged_z-prev_z]))

        # don't move at all if only small changes
        if (error < 0.001):
            filtered.pose.position.x = prev_x
            filtered.pose.position.y = prev_y
            filtered.pose.position.z = prev_z
            filtered.pose.orientation.x = prev_qx
            filtered.pose.orientation.y = prev_qy
            filtered.pose.orientation.z = prev_qz
            filtered.pose.orientation.w = prev_qw

        self.prev_pose = np.array([filtered.pose.position.x, filtered.pose.position.y, filtered.pose.position.z, \
                filtered.pose.orientation.x, filtered.pose.orientation.y, filtered.pose.orientation.z, filtered.pose.orientation.w])

        # publish averaged pose
        self.filtered_pub.publish(filtered)

if __name__ == '__main__':
    rospy.init_node('pose_filter', anonymous=True)
    pose_filter_obj=pose_filter(sys.argv[1], sys.argv[2], sys.argv[3])
    rospy.spin()