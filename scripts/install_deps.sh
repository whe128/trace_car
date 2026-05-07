#!/bin/bash
# install_deps.sh
set -e

ROS_DISTRO=humble

echo "==== apt update ===="
sudo apt update

echo "==== ROS2 dependencies ===="
sudo apt install -y \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-rclpy \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-rosidl-default-generators

echo "==== Gazebo ===="
sudo apt install -y \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-plugins

echo "==== Controllers ===="
sudo apt install -y \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-velocity-controllers \
    ros-$ROS_DISTRO-position-controllers

echo "==== Robot ===="
sudo apt install -y \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-xacro

echo "==== Python ===="
pip3 install \
    opencv-python \
    numpy \
    tkinter

echo "==== cv_bridge ===="
sudo apt install -y \
    ros-$ROS_DISTRO-cv-bridge

echo "==== finish ===="
echo "source /opt/ros/$ROS_DISTRO/setup.bash"
echo "cd ~/ros2_ws && colcon build"
