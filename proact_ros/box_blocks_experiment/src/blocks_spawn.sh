#!/bin/sh

# z_height =$(1.0526)
rosrun gazebo_ros spawn_model -param red_block_description -urdf -model redcylinder1 -x $(rosparam get red_block1_x) -y $(rosparam get red_block1_y) -z 1.0526 -R 0.0 -P 0.0 -Y 0.0
rosrun gazebo_ros spawn_model -param blue_block_description -urdf -model bluecylinder1 -x $(rosparam get blue_block1_x) -y $(rosparam get blue_block1_y) -z 1.0526 -R 0.0 -P 0.0 -Y 0.0
rosrun gazebo_ros spawn_model -param red_block_description -urdf -model redcylinder2 -x $(rosparam get red_block2_x) -y $(rosparam get red_block2_y) -z 1.0526 -R 0.0 -P 0.0 -Y 0.0
rosrun gazebo_ros spawn_model -param blue_block_description -urdf -model bluecylinder2 -x $(rosparam get blue_block2_x) -y $(rosparam get blue_block2_y) -z 1.0526 -R 0.0 -P 0.0 -Y 0.0