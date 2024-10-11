#!/bin/sh

# set -x
# Ask for all this info in the beginning so that mistakes can be corrected without sunk cost
read -p "Enter participant number: " participant
read -p "Enter mode 1/2/3/4: " mode
read -p "Enter trial number: " trial
read -p "Press 0 if using large brace, 1 if medium: " brace
read -p "Are we logging this? 0/1 " log

read -p "Make sure the hololens is calibrated, emg armband is calibrated, shoulder brace is on, previous apps are all closed, then press enter to continue" input

# start listening for hololens connection
read -p "Press enter to start listening for hololens connection" input
gnome-terminal --tab -- roslaunch file_server ros_sharp_communication.launch && xdotool key alt+1

# start publishing emg inputs 
gnome-terminal --tab -- rosrun libemg_ros classify_rosnode.py $participant $mode $trial && xdotool key alt+1

sleep 3

read -p "Check if the armband connection worked, then press enter if it did or restart the node if not" input

read -p "Start hololens app. Once connected, press enter to start simulation" input
gnome-terminal --tab -- bash -c 'roslaunch mpl_control mpl_sim.launch 1> >(grep -v LinkState)' && xdotool key alt+1
gnome-terminal --tab -- bash -c rqt && xdotool key alt+1

# # in a new terminal, listen for mocap
gnome-terminal --tab -- roslaunch shoulder_localization mocap_comm.launch brace_size_M:=$brace && xdotool key alt+1

read -p "Wait till sim is up. Then ask the participant to sit upright, and press enter to continue" input
# set rosworld height in room
gnome-terminal --tab -- rosrun shoulder_localization adjust_height_node.py && xdotool key alt+1

# set up frames
gnome-terminal --tab -- roslaunch shoulder_localization holoworld_with_mocap.launch && xdotool key alt+1

read -p "Ask the participant to be where head is trackable, but not where the table is going to be, then press enter to continue" input
# fix reference calibration
gnome-terminal --tab -- rosrun shoulder_localization fix_world_client.py && xdotool key alt+1

# track shoulder with arm base
gnome-terminal --tab -- roslaunch box_blocks_experiment arm_pose_manager.launch && xdotool key alt+1

# spawn table
gnome-terminal --tab -- roslaunch box_blocks_experiment table_spawn.launch && xdotool key alt+1

# wait 3 seconds 
sleep 3

# spawn blocks
gnome-terminal --tab -- roslaunch box_blocks_experiment blocks_spawn.launch participant:=$participant trial:=$trial && xdotool key alt+1

sleep 3

# set up all grasping-related scripts
gnome-terminal --tab -- roslaunch box_blocks_experiment grasp_setup_all_modes.launch && xdotool key alt+1

# whichever mode the participant wants to play in 
case $mode in
    1) 
    gnome-terminal --tab -- roslaunch box_blocks_experiment modeA_joint_jog.launch;;
    2) 
    gnome-terminal --tab -- roslaunch box_blocks_experiment modeB_ee_jog.launch;;
    3) 
    gnome-terminal --tab -- roslaunch box_blocks_experiment modeC_gaze.launch mode:=0;;
    4) 
    gnome-terminal --tab -- roslaunch box_blocks_experiment modeC_gaze.launch mode:=1;;
esac

case $log in
    0) 
    read -p "Not logging anything. Press enter to end the run" input
    ;;
    
    1) 
    read -p "Press enter to start logging" input
    gnome-terminal --tab -- roslaunch box_blocks_experiment logging.launch participant:=$participant mode:=$mode trial:=$trial && xdotool key alt+1
    # get current time
    current_time=$(date +%Y-%m-%d-%H-%M-%S)
    gnome-terminal --tab -- gz log -d 1 && xdotool key alt+1
    read -p "Press enter to record the start of the experiment" input
    # publish a log message to rosout
    gnome-terminal --tab -- rostopic pub /rosout rosgraph_msgs/Log '{header: {stamp: now}, level: 2, name: "start-stop", msg: "Start", file: "", function: "N/A", line: 0, topics: []}' && xdotool key alt+1
    read -p "Press enter to end the experiment" input
    read -p "Press enter again to log a stop message" input
    gnome-terminal --tab -- rostopic pub /rosout rosgraph_msgs/Log '{header: {stamp: now}, level: 2, name: "start-stop", msg: "Stop", file: "", function: "N/A", line: 0, topics: []}' && xdotool key alt+1
    ;;

esac

read -p "Are you sure you want to kill all nodes?" input

rosnode kill -a
killall roscore

case $log in 
    0) 
    echo "Okay!";;

    1) 
    gnome-terminal --tab -- gz log -d 0 && xdotool key alt+1;;

esac

