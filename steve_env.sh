#!/usr/bin/env bash
# steve_env.sh — connect THIS dev-PC container to Steve's onboard-PC ROS 2 graph.
#
#   source /home/ws/steve_env.sh          # set up network + env in this shell
#   source /home/ws/steve_env.sh --check  # just report status, change nothing
#
# Nothing on the onboard robot PC is touched. All changes are local to this
# container and are NOT persistent (live `ip` address + shell env) — re-source
# after a container restart or a cable replug.
#
# Background (found 2026-09-10): the onboard PC (mmo-700-uni-bonn-2204) publishes
# ROS 2 on the 192.168.60.0/24 subnet — DDS discovery multicast is sourced from
# 192.168.60.90 (its enp2s0 NIC). It is NOT reachable for ROS on 192.168.1.0/24
# even though that NIC answers ARP for 192.168.1.10 (a dead-end address).
# Onboard ROS settings: ROS_DOMAIN_ID=74, RMW=rmw_fastrtps_cpp, LOCALHOST_ONLY=0.

# ---- config -----------------------------------------------------------------
STEVE_IFACE="${STEVE_IFACE:-enp3s0}"        # dev-PC NIC on the base LAN cable
STEVE_MYIP="${STEVE_MYIP:-192.168.60.50}"   # address to claim (must be free)
STEVE_CIDR=24
STEVE_ONBOARD="${STEVE_ONBOARD:-192.168.60.90}"
STEVE_DOMAIN_ID=74
STEVE_RMW=rmw_fastrtps_cpp
# ---------------------------------------------------------------------------

_steve_status() {
  local ip carrier
  carrier=$(cat "/sys/class/net/$STEVE_IFACE/carrier" 2>/dev/null || echo "?")
  ip=$(ip -4 -br addr show "$STEVE_IFACE" 2>/dev/null | awk '{$1=$2="";print}' | xargs)
  echo "iface     : $STEVE_IFACE  (carrier=$carrier)  addr: ${ip:-<none>}"
  if ping -c1 -W1 "$STEVE_ONBOARD" >/dev/null 2>&1; then
    echo "onboard   : $STEVE_ONBOARD reachable"
  else
    echo "onboard   : $STEVE_ONBOARD NOT reachable  (cable? wrong iface? set STEVE_IFACE=)"
  fi
  echo "ROS env   : DOMAIN_ID=${ROS_DOMAIN_ID:-unset}  LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}  RMW=${RMW_IMPLEMENTATION:-unset}"
}

if [ "$1" = "--check" ]; then
  _steve_status
  return 0 2>/dev/null || exit 0
fi

# --- 1. network: put STEVE_IFACE on the onboard subnet (idempotent) ---------
if [ "$(cat "/sys/class/net/$STEVE_IFACE/carrier" 2>/dev/null)" != "1" ]; then
  sudo ip link set "$STEVE_IFACE" up 2>/dev/null
  sleep 1
fi
if ! ip -4 addr show "$STEVE_IFACE" 2>/dev/null | grep -q "inet ${STEVE_MYIP}/"; then
  sudo ip addr add "${STEVE_MYIP}/${STEVE_CIDR}" dev "$STEVE_IFACE" 2>/dev/null \
    && echo "added ${STEVE_MYIP}/${STEVE_CIDR} to $STEVE_IFACE" \
    || echo "could not add ${STEVE_MYIP} (already present, or no sudo)"
fi

# --- 2. ROS 2 discovery must match the onboard PC --------------------------
export ROS_DOMAIN_ID=$STEVE_DOMAIN_ID
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=$STEVE_RMW

# --- 3. source ROS (only if not already on PATH) --------------------------
[ -z "$ROS_DISTRO" ] && [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /home/ws/install/setup.bash ] && source /home/ws/install/setup.bash

# --- 4. reset the daemon so it picks up the new domain/env ----------------
ros2 daemon stop >/dev/null 2>&1

echo "----------------------------------------------------------------"
_steve_status
echo "----------------------------------------------------------------"
echo "check:  ros2 topic list   |   ros2 topic echo /joint_states --once"
