#!/bin/bash
echo "Remember to run this file with: source ros2_localhost_connect.sh"
export ROS_LOCALHOST_ONLY=1
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
ros2 daemon stop &&
ros2 daemon start
echo "ROS2 configured to use localhost only."

# Determine script directory (works when sourced)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="$SCRIPT_DIR/msgs_ws"

if [ -f "$ROS2_WS/install/setup.bash" ]; then
	source "$ROS2_WS/install/setup.bash"
	echo "Sourced dls2_interface ROS2 workspace: $ROS2_WS"
else
	echo "Warning: $ROS2_WS/install/setup.bash not found."
    
	# Build the workspace (assume the workspace directory exists and colcon is available)
	echo "Building ROS2 workspace with colcon in: $ROS2_WS"
	(cd "$ROS2_WS" && colcon build) || {
		echo "colcon build failed."
		return 1
	}
	if [ -f "$ROS2_WS/install/setup.bash" ]; then
		source "$ROS2_WS/install/setup.bash"
		echo "Sourced ROS2 workspace after build: $ROS2_WS"
	else
		echo "Build finished but $ROS2_WS/install/setup.bash still not found."
		return 1
	fi
fi