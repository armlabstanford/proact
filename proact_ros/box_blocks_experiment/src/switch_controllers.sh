#!/bin/sh
# switch to joint-level controllers

## if controlling arm dofs two at a time (e.g. with joystick)
# rosservice call /mpl_right_arm/controller_manager/load_controller "name: 'shoulder_controller'" 
# rosservice call /mpl_right_arm/controller_manager/load_controller "name: 'elbow_controller'" 
# rosservice call /mpl_right_arm/controller_manager/load_controller "name: 'wrist_zx_controller'" 
# rosservice call /mpl_right_arm/controller_manager/load_controller "name: 'wrist_y_controller'" 
# rosservice call /mpl_right_arm/controller_manager/switch_controller "{start_controllers: ['shoulder_controller', 'elbow_controller', 'wrist_zx_controller', 'wrist_y_controller'], stop_controllers: ['arm_controller'], strictness: 2}"

## if controlling arm dofs one at a time
rosservice call /mpl_right_arm/controller_manager/load_controller "name: 'j0_controller'" 
rosservice call /mpl_right_arm/controller_manager/load_controller "name: 'j1_controller'"
rosservice call /mpl_right_arm/controller_manager/load_controller "name: 'j2_controller'"
rosservice call /mpl_right_arm/controller_manager/load_controller "name: 'j3_controller'"
rosservice call /mpl_right_arm/controller_manager/load_controller "name: 'j4_controller'"
rosservice call /mpl_right_arm/controller_manager/load_controller "name: 'j5_controller'"
rosservice call /mpl_right_arm/controller_manager/load_controller "name: 'j6_controller'"
rosservice call /mpl_right_arm/controller_manager/switch_controller "{start_controllers: ['j0_controller', 'j1_controller', 'j2_controller', 'j3_controller', 'j4_controller', 'j5_controller', 'j6_controller'], stop_controllers: ['arm_controller'], strictness: 2}"
