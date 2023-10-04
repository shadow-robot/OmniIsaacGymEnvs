#!/bin/bash
set -e
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt update
apt install -y ros-noetic-ros-base
apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential tf2-ros
apt install python3-rosdep
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash
rosdep init
rosdep update
/isaac-sim/python.sh -m pip install rospkg

