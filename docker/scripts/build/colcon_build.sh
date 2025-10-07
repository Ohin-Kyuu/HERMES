#!/usr/bin/env bash
set -e 

USER=$1
ROS_WS=/home/${USER}/hermes_ws

. "/opt/ros/humble/setup.sh"
cd ${ROS_WS}
colcon build --symlink-install
echo "source ${ROS_WS}/install/setup.bash" >> /home/${USER}/.bashrc