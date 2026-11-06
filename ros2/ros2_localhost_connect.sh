#!/bin/bash
echo "Remember to run this file with: source ros2_localhost_connect.sh"
export ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI
unset RMW_IMPLEMENTATION
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT

# Restart the ros2 CLI daemon so it picks up the settings above. A daemon left
# over from another profile (e.g. cyclone bound to an interface that went down)
# can stop answering, and `ros2 daemon stop` then waits on it forever.
timeout -k 2 5 ros2 daemon stop || {
    echo "ros2 daemon not responding, killing it"
    pkill -KILL -f "ros2cli\.daemon\.daemonize .*--ros-domain-id ${ROS_DOMAIN_ID:-0} "
    sleep 1
}
ros2 daemon start
echo "ROS2 configured to use localhost only."