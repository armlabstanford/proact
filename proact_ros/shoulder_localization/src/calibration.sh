#!/bin/sh

# set -x
# Ask for all this info in the beginning so that mistakes can be corrected without sunk cost
read -p "If you checked motion capture is streaming and able to connect to ROS, place the HoloLens on the rig, then press enter to start listening for HoloLens connection" input
gnome-terminal --tab -- bash -c 'roslaunch file_server ros_sharp_communication.launch' && xdotool key alt+1
read -p "Start the HoloLens app and press enter to start mocap communication" input
gnome-terminal --tab -- bash -c 'roslaunch shoulder_localization mocap_comm.launch' && xdotool key alt+1
read -p "Once the app is connected to ROS, put the HoloLens on, and manually align the reference board" input
read -p "Once aligned, replace the HoloLens on the rig and press enter" input
gnome-terminal --tab -- bash -c 'roslaunch shoulder_localization find_head_mocap_tf.launch' && xdotool key alt+1
read -p "Press enter to see the tf" input
gnome-terminal --tab -- bash -c 'rosrun tf tf_echo holohead HololensSG' && xdotool key alt+1
read -p "Go to the last terminal, copy the tf and press enter to stop" input
read -p "Are you sure you got the tf?" input
rosnode kill -a
killall roscore



