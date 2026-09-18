#!/usr/bin/env bash
# onboard_probe.sh — run this ON STEVE'S ONBOARD PC (not the devcontainer).
#
# Copy it over and run, e.g.:
#     scp /home/ws/onboard_probe.sh neobotix@192.168.60.90:/tmp/
#     ssh neobotix@192.168.60.90 'bash /tmp/onboard_probe.sh'
# or paste it into an ssh session. Then copy /tmp/onboard_probe_out.txt back
# so Claude can read it (e.g. scp ... :/tmp/onboard_probe_out.txt /home/ws/diag_out/).
#
# Read-only. Tells us the onboard PC's ROS discovery settings + what the arm
# is actually publishing, so the devcontainer can be matched to it.

set -u
OUT=/tmp/onboard_probe_out.txt
exec > >(tee "$OUT") 2>&1

sec() { echo; echo "===================== $* ====================="; }
run() { echo "\$ $*"; timeout 20 bash -c "$*" 2>&1; echo "(exit $?)"; }

echo "onboard_probe.sh  $(date +%Y%m%d_%H%M%S)"
run "id; hostname; uname -a"

sec "1. ROS env in THIS shell"
env | grep -Ei 'ROS|RMW|FASTRTPS|CYCLONE|DDS' | sort
echo "ROS_DOMAIN_ID      = ${ROS_DOMAIN_ID:-<unset>}"
echo "ROS_LOCALHOST_ONLY = ${ROS_LOCALHOST_ONLY:-<unset>}"
echo "RMW_IMPLEMENTATION = ${RMW_IMPLEMENTATION:-<unset>}"

sec "2. what the autostart / bringup sets"
for f in ~/ROS_AUTOSTART.sh /home/neobotix/ROS_AUTOSTART.sh ~/.bashrc ~/steve_ros2_ws/install/setup.bash; do
  [ -f "$f" ] && { echo "--- $f ---"; grep -nE 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION|CYCLONEDDS_URI|FASTRTPS|ROS_DISCOVERY|ROS_STATIC_PEERS|export ROS' "$f"; }
done

sec "3. env of the RUNNING bringup process"
pid=$(pgrep -f 'hardware_bringup|ros2 launch' | head -1)
echo "bringup-ish pid: ${pid:-<none>}"
[ -n "${pid:-}" ] && tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -Ei 'ROS|RMW|DDS' | sort

sec "4. network interfaces + addresses"
ip -brief addr 2>/dev/null || ip addr 2>/dev/null || hostname -I
echo "--- routes ---"
ip route 2>/dev/null
echo "--- multicast route ---"
ip route 2>/dev/null | grep -E '224\.0\.0\.0/4' || echo "(no explicit multicast route)"

sec "5. arm reachable from onboard PC (the old blocker)"
for ip in 192.168.1.102 192.168.1.101; do
  echo "--- $ip ---"
  ping -c 2 -W 2 "$ip" 2>&1
done

sec "6. ros2 topics/nodes visible ON the onboard PC"
source /opt/ros/*/setup.bash 2>/dev/null
[ -f ~/steve_ros2_ws/install/setup.bash ] && source ~/steve_ros2_ws/install/setup.bash 2>/dev/null
timeout 10 ros2 daemon stop 2>/dev/null
echo "--- topic list ---"
timeout 20 ros2 topic list -t --no-daemon 2>&1
echo "--- node list ---"
timeout 20 ros2 node list --no-daemon 2>&1
echo "--- /joint_states one sample ---"
timeout 8 ros2 topic echo /joint_states --once --no-daemon 2>&1
echo "--- /dynamic_joint_states one sample (arm controllers) ---"
timeout 8 ros2 topic echo /dynamic_joint_states --once --no-daemon 2>&1
echo "--- controllers (via topic, list_controllers is known to crash here) ---"
timeout 20 ros2 control list_controllers --no-daemon 2>&1 || true

sec "DONE"
echo "Output: $OUT  — copy it back to the laptop for Claude."
