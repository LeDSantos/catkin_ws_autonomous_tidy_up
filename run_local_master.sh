#!/bin/bash

echo "Set master at localhost"
export ROS_MASTER_URI=http://localhost:11311
# export ROS_HOSTNAME=127.0.0.1
export ROS_IP=127.0.0.1
echo $ROS_MASTER_URI
# echo $ROS_HOSTNAME
echo $ROS_IP
#in case of problems with the first catkin_make: export CMAKE_PREFIX_PATH=/home/your_user/your_path/catkin_ws_autonomous_tidy_up/devel:/tiago_public_ws/devel