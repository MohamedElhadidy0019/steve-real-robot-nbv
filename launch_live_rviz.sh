#!/bin/bash
# launch_live_rviz.sh — RViz on the dev PC mirroring Steve's live real arm pose.
#
#   bash /home/ws/launch_live_rviz.sh
#
# Pure viewer: subscribes to /robot_description, /tf, /tf_static from the
# onboard PC (domain 74). No nodes started, nothing published, nothing that
# can move or command the real robot. Arm-only view (base/pan-tilt/wheels
# hidden in the saved config) — see steve_live_rviz.rviz.

set -e

# 1. clear stale ros2cli daemons (recurring gotcha on this box — a daemon left
#    over from a different ROS_DOMAIN_ID answers with a stale/empty graph)
pkill -9 -f ros2cli.daemon 2>/dev/null || true

# 2. connect to the onboard PC's ROS 2 graph (domain 74, IP on its subnet)
source /home/ws/steve_env.sh

# 3. launch RViz with the arm-only config
RVIZ_CFG=/home/ws/steve_live_rviz.rviz
if [ -f "$RVIZ_CFG" ]; then
  exec rviz2 -d "$RVIZ_CFG"
else
  echo "warning: $RVIZ_CFG not found, launching bare rviz2" >&2
  exec rviz2
fi
