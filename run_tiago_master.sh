#!/bin/bash

echo "Set master at tiago"
export ROS_MASTER_URI=http://tiago-135c:11311
# ROS_HOSTNAME do not exist
# unset ROS_HOSTNAME
export ROS_IP=robot-laptop-nb.uio.no
echo $ROS_MASTER_URI
# echo $ROS_HOSTNAME
echo $ROS_IP