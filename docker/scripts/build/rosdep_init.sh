#!/usr/bin/env bash
set -e 

USER=$1
ROS_WS=/home/${USER}/hermes_ws

. "/opt/ros/humble/setup.sh"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "[rosdep] initializing..."
    sudo rosdep init
else
    echo "[rosdep] already initialized, skip"
fi

apt-get update
rosdep update
rosdep install \
    --rosdistro humble \
    --from-paths ${ROS_WS}/src \
    --ignore-src \
    --skip-keys "livox_ros_driver2" \
    -y