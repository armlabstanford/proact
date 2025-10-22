# 🦾 Prosthetic Arm Control Testbed 

### [Shivani Guptasarma](https://arm.stanford.edu/people/shivani-guptasarma), [Monroe Kennedy III](https://monroekennedy3.com/)

This page describes how to use ProACT to set up a study on intelligent control for prosthetic arms.

![](https://github.com/armlabstanford/armlabstanford.github.io/blob/master/static/proact/images/proact_splash.png)

[![Project](https://img.shields.io/badge/Project_Page-ProACT-green)](https://armlabstanford.github.io/proact)
[![ArXiv](https://img.shields.io/badge/Arxiv-ProACT-red)](https://arxiv.org/abs/2407.05025) 

Tested on the following setup:
- Ubuntu 20.04 
- ROS Noetic
- Hololens 2 with Windows Holographic for Business OS build 22621.1266
- Unity 2020.3.26.f1
- Mixed Reality Toolkit 2.7.3.0
- OptiTrack motion capture PrimeX series with Motive Tracker 3.0.1
- OyMotion GForce Pro+ EMG band
- LibEMG v0.0.1
- Python3.8

# Setup before inviting participants 

## Network

Connect the Linux computer running ROS, Windows computer running Unity, HoloLens 2, and whichever computer is running Motive, all to the same WiFi network and note the IP addresses.

## Installation

1. Install [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu) and [make a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 
2. Clone the ROS package `proact_ros` from this repository into the `src` directory of your workspace. 
3. Install dependencies using [rosdep](http://wiki.ros.org/rosdep) and run
```
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control ros-noetic-moveit ros-noetic-rviz-visual-tools ros-noetic-moveout-visual-tools ros-noetic-vrpn-client-ros
sudo apt-get install xdotool
```
4. Copy the file_server package from the UWP fork of ROS#: https://github.com/EricVoll/ros-sharp into the workspace and install the rosbridge server: `sudo apt-get install ros-noetic-rosbridge-server`.
5. Clone packages from https://github.com/JenniferBuehler/general-message-pkgs and https://github.com/JenniferBuehler/gazebo-pkgs into the workspace.
6. Install [libemg](https://libemg.github.io/libemg/index.html) : `pip3 install libemg==0.0.1`. Version 1.0.2 (latest at the time of this writing) does not support the OyMotion band.
7. Clone [libemg-ros](https://github.com/armlabstanford/libemg-ros/tree/main) into the catkin workspace and install its dependencies.
8. In `shoulder_localization/launch/mocap_comm.launch`, replace the server IP with the IP of the computer running Motive.
9. Extend the configuration of the workspace `catkin config --extend /opt/ros/noetic/` and build it.

## Motion capture 

Attach motion capture markers and create Motive rigid bodies for: 
1. A shoulder brace 
2. The HoloLens 2
3. A reference object.
   
In `shoulder_localization/launch/mocap_comm.launch`, these are named Shoulderpad/ShoulderpadL (we had two sizes), HololensSG, and Clipboard. Replace these names in the launch file, and everywhere else they appear, if using different names in Motive.

Remember to export these as Motive assets to avoid repeating calibration. 

<img src="https://github.com/armlabstanford/armlabstanford.github.io/blob/master/static/proact/images/mocap_brace.jpg" width="200">    <img src="https://github.com/armlabstanford/armlabstanford.github.io/blob/master/static/proact/images/mocap_hololens.jpg" width="200">    <img src="https://github.com/armlabstanford/armlabstanford.github.io/blob/master/static/proact/images/mocap_ref.jpg" width="200">

:warning: 
- HoloLens markers need to be on the top of the visor (so that they are fixed relative to the IMU and not occluding any cameras), and non-collinear as usual. 
- Putting too many markers on the surface will cause them to occlude each other.  
- Note that turning the visor up after this might damage markers. 

In Motive, open network settings from the bottom right and enable VRPN streaming.

:memo: The first time this setup is used, the coordinate frame attached to the shoulder (where the base of the arm appears) will likely need to be manually adjusted in Motive, to attain some intersection of believability and visibility. 
If the tracked frames jitter too much, it is best to add more markers and/or adjust the cameras and recalibrate so that continuous visibility is assured. If that does not solve the problem, Motive offers a smoothing setting (select the Asset and scroll down in the Properties panel) that can be turned up for both the HoloLens and and shoulder brace. 

## Windows

This part of the setup may require some downgrades on your system, since the ROS# UWP fork was written for older versions of Unity, which work with older versions of Visual Studio.

1. Install [Unity 2020](https://unity.com/releases/editor/archive) and open the Unity project from this repository in the Unity editor. 
2. The ROS Connector game object has a ROS Connector script attached to it. Replace the ROS bridge server URL with the Linux computer IP address, keeping the port 9090.
3. Build the app for Universal Windows Platform. 
4. Install Visual Studio 2019. Open the .sln file generated by the Unity build in VS2019, and set up for UWP deployment: Release configuration, ARM64 platform, Remote Machine. In Project>Properties>Debugging, enter the HoloLens IP in the machine name field. 
5. Deploy to HoloLens and give the app permissions.

At first, the arm, box and blocks will appear above your head somewhere. To match motion capture and Unity worlds, we need to measure the transform between the head frame measured by Unity and the reference frame attached to the HoloLens in Motive, as described below. 

## Calibration for localization

1. Find a way to repeatedly place the HoloLens in a fixed pose, for a fixed headband circumference. We built a constraining rig from some old PC parts and packaging:

<img src="https://github.com/armlabstanford/armlabstanford.github.io/blob/master/static/proact/images/rig_side.jpg" width="200">    <img src="https://github.com/armlabstanford/armlabstanford.github.io/blob/master/static/proact/images/rig_top_rotated.jpg" width="200">

2. Disable both the RosConnector > HideFrames scripts in Unity before building and deploying. 

3. Run `shoulder_localization/src/calibration.sh` in a terminal while the HoloLens is on the rig and do as prompted:
- Once the app has started and is connected to ROS, wear the HoloLens to manually align the reference mocap frame with the virtual coordinate frame that appears 30 cm in front of the Unity world frame.

<img src="https://github.com/armlabstanford/armlabstanford.github.io/blob/master/static/proact/images/calib_full.jpg" height="200">    <img src="https://github.com/armlabstanford/armlabstanford.github.io/blob/master/static/proact/images/calib_virtual.jpg" height="200">

- Replace the HoloLens on the rig and get the transform between the Unity head frame (coincident, at this pose, with the Unity head frame) and HoloLens mocap frame. 
- Paste these values into the initialization section (`self.pos_m2c` and `temp`) of `shoulder_localization/src/holoworld_with_mocap.py`. Note that tf puts q0 after q1, q2, q3, while `pyquaternion` expects them in the order q0, q1, q2, q3. The script arranges them so that you can paste values directly from the terminal.

# With participants 

## EMG

1. Have participants put on the EMG band snugly on the forearm, and switch on the band. 
2. Navigate to `libemg-ros` and run `python3 train.py`, clicking "Train Model" to train gestures. Explain that the training gestures need to be as independent of each other as possible in the way they are performed.

<img src="https://github.com/armlabstanford/armlabstanford.github.io/blob/master/static/proact/videos/emg_train.gif" height="300">

4. Close the training window to return to the main page, clicking "Classify" to check the quality of training and retrain if necessary.

<img src="https://github.com/armlabstanford/armlabstanford.github.io/blob/master/static/proact/videos/emg_test.gif" height="300">

## HoloLens

1. Explain how to open and close the main menu.
2. Guide participants to Settings > System > Calibration > Run eye calibration and let them follow the instructions. 

## Shoulder tracking 

Secure the brace with mocap markers on the participant's shoulder.

## Running experiments

Run `box_and_blocks_experiment/src/master.sh`. 
