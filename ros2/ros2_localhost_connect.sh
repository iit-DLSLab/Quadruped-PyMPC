#!/bin/bash
echo "Remember to run this file with: source ros2_localhost_connect.sh"

# Profilo simulazione locale.
export ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI
unset RMW_IMPLEMENTATION
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
ros2 daemon stop &&
ros2 daemon start
echo "ROS2 configured to use localhost only."