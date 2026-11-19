#!/usr/bin/env bash
set -e
printf '\n=== System ===\n'
uname -a
/usr/bin/python3 --version
printf '\n=== ROS environment ===\n'
if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi
if [ -f "$HOME/smartcart_ws/install/setup.bash" ]; then source "$HOME/smartcart_ws/install/setup.bash"; fi
printf 'ROS_DISTRO=%s\nROS_DOMAIN_ID=%s\nRMW_IMPLEMENTATION=%s\n' "${ROS_DISTRO:-unset}" "${ROS_DOMAIN_ID:-0 (default)}" "${RMW_IMPLEMENTATION:-default}"
/usr/bin/python3 -c 'import rclpy; from sensor_msgs.msg import LaserScan; print("ROS Python imports: OK")' || true
printf '\n=== Serial device (owned only by sllidar_ros2) ===\n'
ls -l /dev/ttyUSB* 2>/dev/null || true
printf '\n=== Addresses ===\n'
hostname -I || true
printf '\n=== Scan topic, maximum 8 seconds ===\n'
if command -v ros2 >/dev/null; then timeout 8s ros2 topic info /scan --verbose || true; fi
printf '\n=== Radar listener ===\n'
ss -ltn '( sport = :8765 )' || true
printf '\nRead-only checks finished. No drivers or services were changed.\n'
