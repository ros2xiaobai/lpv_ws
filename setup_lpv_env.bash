#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash

if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
  source "$HOME/catkin_ws/devel/setup.bash"
fi

if [ -f "$HOME/PX4_Firmware/Tools/setup_gazebo.bash" ]; then
  source "$HOME/PX4_Firmware/Tools/setup_gazebo.bash" \
    "$HOME/PX4_Firmware" \
    "$HOME/PX4_Firmware/build/px4_sitl_default"
fi

if [ -f "$HOME/LPV_ws/devel/setup.bash" ]; then
  source "$HOME/LPV_ws/devel/setup.bash"
fi

export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$HOME/PX4_Firmware"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$HOME/PX4_Firmware/Tools/sitl_gazebo"

export LPV_WS="$HOME/LPV_ws"
