#!/usr/bin/env python

##########################################################

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


deg2rad = tau/360.0

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
   
###########################################################

# initialize node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("picker_placer", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "mpl_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)   

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

# # We get the joint values from the group and change some of the values:
# joint_goal = move_group.get_current_joint_values()
# joint_goal[3] += tau / 13

# print(joint_goal)

# # The go command can be called with joint values, poses, or without any
# # parameters if you have already set the pose or joint target for the group
# move_group.go(joint_goal, wait=True)

# # Calling ``stop()`` ensures that there is no residual movement
# move_group.stop()

############################################################################

listener = tf.TransformListener()
done = False

while not done:
        try:
            (trans,rot) = listener.lookupTransform(planning_frame, '/cylinder', rospy.Time(0))
            done = True
            print("got it!")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("missed base to cylinder tf")

pose_goal = geometry_msgs.msg.Pose()
# pose_goal.header.frame_id = "world"

pose_goal.position.x = trans[0] - 0.051 #- 0.063
pose_goal.position.y = trans[1] - 0.050 #- 0.057 # 0.0
pose_goal.position.z = trans[2] + 0.125 #0.098 # 0.09 #+ 0.068 # + 0.025 + 0.05  #1.07
# pose_goal.orientation.x = 0.086 # rot[0] # 0.008
# pose_goal.orientation.y = 0.140 # rot[1] # 0.008 
# pose_goal.orientation.z = 0.830 # rot[2] # 0.842 
# pose_goal.orientation.w = 0.533 # rot[3] # 0.540 
pose_goal.orientation.x = 0.0 #rot[0] # 0.008
pose_goal.orientation.y = 0.0 #rot[1] # 0.008 
pose_goal.orientation.z = 0.841 #rot[2] # 0.842 
pose_goal.orientation.w = 0.540 #rot[3] # 0.540 
# 0.131, 0.208, 0.815, 0.525
move_group.set_pose_target(pose_goal)

# `go()` returns a boolean indicating whether the planning and execution was successful.
success = move_group.go(wait=True)
print(success)
# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets().
move_group.clear_pose_targets()

###########################################################################

# # group_name = "mpl_hand"
# # move_group = moveit_commander.MoveGroupCommander(group_name)   

# # We can get the name of the reference frame for this robot:
# planning_frame = move_group.get_planning_frame()
# print("============ Planning frame: %s" % planning_frame)

# # We can also print the name of the end-effector link for this group:
# eef_link = move_group.get_end_effector_link()
# print("============ End effector link: %s" % eef_link)

# # # We can get a list of all the groups in the robot:
# # group_names = robot.get_group_names()
# # print("============ Available Planning Groups:", robot.get_group_names())

# # # Sometimes for debugging it is useful to print the entire state of the
# # # robot:
# # print("============ Printing robot state")
# # print(robot.get_current_state())
# # print("")

# joint_goal = move_group.get_current_joint_values()
# print(joint_goal)

# joint_goal[0] = -10.0*deg2rad
# # # joint_goal[1] = 81.0*deg2rad
# # # joint_goal[2] = 13.0*deg2rad
# # # joint_goal[3] = 14.0*deg2rad
# # # joint_goal[4] = 0.0*deg2rad
# # # joint_goal[5] = 72.0*deg2rad
# # # joint_goal[6] = 22.0*deg2rad
# # # joint_goal[7] = 0.0*deg2rad
# # # joint_goal[8] = 11.0*deg2rad
# # # joint_goal[9] = 17.0*deg2rad
# # # joint_goal[10] = 3.0*deg2rad
# # # joint_goal[11] = 1.0*deg2rad
# # # joint_goal[12] = 20.0*deg2rad
# # # joint_goal[13] = 80.0*deg2rad
# # # joint_goal[14] = 0.0*deg2rad
# # # joint_goal[15] = 0.0*deg2rad
# # # joint_goal[16] = 108.0*deg2rad
# # # joint_goal[17] = 23.0*deg2rad
# # # joint_goal[18] = 13.0*deg2rad
# # # joint_goal[19] = 18.0*deg2rad

# print(joint_goal)

# # The go command can be called with joint values, poses, or without any
# # parameters if you have already set the pose or joint target for the group
# move_group.go(joint_goal, wait=True)

# # Calling ``stop()`` ensures that there is no residual movement
# move_group.stop()

##########################################################################

# We get the joint values from the group and change some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[6] += 20*deg2rad
# joint_goal = tuple(joint_goal)
print(joint_goal)

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(copy.deepcopy(joint_goal), wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()