#!/bin/bash

set -e
export DEBIAN_FRONTEND noninteractive
export TERM xterm

apt-get update && apt-get install -y -q wget sudo lsb-release # for docker 

#before_install:
sh -c "echo \"deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main\" > /etc/apt/sources.list.d/ros-latest.list"
wget http://packages.ros.org/ros.key -O - | apt-key add -
apt-get update && apt-get install -y -q python-catkin-pkg python-rosdep python-wstool ros-kinetic-catkin build-essential
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep init
rosdep update

#install:
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cd ~/catkin_ws/src
git clone https://github.com/shinselrobots/body_tracker_msgs.git
git clone https://github.com/shinselrobots/safety_control.git
git clone https://github.com/shinselrobots/robot_speech.git
git clone https://github.com/shinselrobots/robot_behavior.git
git clone -b master_shinselrobots https://github.com/shinselrobots/rplidar_ros.git
git clone -b kinetic-devel-shinselrobots https://github.com/shinselrobots/realsense_samples_ros.git
ln -s $CI_SOURCE_PATH .

#before_script:
cd ~/catkin_ws
rosdep install -q -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO

#script:
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws
catkin_make
