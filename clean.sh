#!/bin/bash
# Kill every ROS process + reset the ros2 daemon, so launch_mock.sh starts from a clean graph.
# Run this whenever you see duplicate /controller_manager nodes or xmlrpc !rclpy.ok() errors.

echo "--- before ---"
ps aux | grep -E 'ros2_control_node|robot_state_publisher|controller_manager|spawner|rviz|ros2 launch|_ros2cli' | grep -v grep

pkill -9 -f ros2_control_node
pkill -9 -f robot_state_publisher
pkill -9 -f 'controller_manager/spawner'
pkill -9 -f 'ros2 launch'
pkill -9 -f rviz
pkill -9 -f _ros2cli_daemon
pkill -9 -f 'ros2cli'

# ros2 daemon may already be dead; ignore errors
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start 2>/dev/null

sleep 1
echo "--- after (should be EMPTY) ---"
ps aux | grep -E 'ros2_control_node|robot_state_publisher|controller_manager|spawner|rviz' | grep -v grep
echo "--- done ---"
