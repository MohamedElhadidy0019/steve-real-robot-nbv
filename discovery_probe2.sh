#!/usr/bin/env bash
# discovery_probe2.sh — follow-up. We now know the onboard ROS graph is on
# ROS_DOMAIN_ID=0 and there's a likely DDS-vendor/version mismatch
# ("sequence size exceeds remaining buffer"). This pins down:
#   - who publishes /joint_states (node name / host / GID)
#   - whether switching RMW to CycloneDDS fixes enumeration
#   - whether there's a local IP collision on 192.168.1.10
#   - link state of enp3s0
#
#   bash /home/ws/discovery_probe2.sh
# writes /home/ws/diag_out/discovery_probe2_*.txt   (read-only, no changes)

set -u
OUT=/home/ws/diag_out
mkdir -p "$OUT"
TS=$(date +%Y%m%d_%H%M%S)
LOG="$OUT/discovery_probe2_$TS.txt"
exec > >(tee "$LOG") 2>&1

sec() { echo; echo "===================== $* ====================="; }
run() { echo "\$ $*"; timeout 25 bash -c "$*" 2>&1; echo "(exit $?)"; }

export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=0
timeout 10 ros2 daemon stop >/dev/null 2>&1
echo "discovery_probe2  $TS   DOMAIN_ID=0  LOCALHOST_ONLY=0"

sec "1. local ROS-ish processes (is anything publishing locally?)"
run "ps -eo pid,user,cmd | grep -Ei 'ros2|_ros2_daemon|controller_manager|robot_state_pub|ur_robot_driver|ur_ros2|dashboard|mock|launch' | grep -v grep"

sec "2. enp3s0 link + duplicate-address check for 192.168.1.10"
run "ip -br addr show enp3s0; ethtool enp3s0 2>/dev/null | grep -E 'Speed|Duplex|Link detected'"
run "cat /sys/class/net/enp3s0/carrier /sys/class/net/enp3s0/speed 2>/dev/null"
# arping -D = duplicate address detection; needs the pkg, harmless if missing
run "arping -D -I enp3s0 -c 3 192.168.1.10"
run "arping -I enp3s0 -c 3 192.168.1.11"
run "arping -I enp3s0 -c 3 192.168.1.102"
run "ip neigh show dev enp3s0"

sec "3. FastRTPS (current RMW): who is on domain 0"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
run "ros2 node list --no-daemon"
run "ros2 topic list -t --no-daemon"
run "ros2 topic info /joint_states --verbose --no-daemon"
run "ros2 topic info /gpio_controller/gpio_states --verbose --no-daemon"
run "ros2 topic echo /joint_states --once --no-daemon"

sec "4. Is CycloneDDS available here?"
run "ros2 pkg prefix rmw_cyclonedds_cpp"
run "dpkg -l 'ros-humble-rmw-cyclonedds-cpp' 2>/dev/null | tail -1"

sec "5. CycloneDDS attempt (only if installed): does enumeration clean up?"
if ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset CYCLONEDDS_URI
  timeout 10 ros2 daemon stop >/dev/null 2>&1
  run "ros2 node list --no-daemon"
  run "ros2 topic list -t --no-daemon"
  run "ros2 topic info /joint_states --verbose --no-daemon"
  run "ros2 topic echo /joint_states --once --no-daemon"
else
  echo "rmw_cyclonedds_cpp NOT installed -> to test:  sudo apt install -y ros-humble-rmw-cyclonedds-cpp"
fi

sec "6. discovery info (FastRTPS)"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
run "timeout 15 ros2 doctor --report 2>/dev/null | sed -n '/TOPIC LIST/,\$p'"

sec DONE
echo "log: $LOG"
