#!/usr/bin/env bash
# discovery_probe.sh — diagnose ROS 2 multi-machine discovery between THIS
# devcontainer (laptop) and Steve's onboard PC, reached over the chassis LAN
# cable (chassis "DHCP" port -> onboard enp0s31f6 = 192.168.60.90/24).
#
# Claude cannot run commands in this container. Run this yourself:
#     bash /home/ws/discovery_probe.sh [ONBOARD_IP]
# then tell Claude; it reads /home/ws/diag_out/discovery_probe_*.txt
#
# Safe: read-only probes only. No node/param changes, no code changes.
# Every ros2 call has a HARD timeout and runs --no-daemon.

set -u

ONBOARD_IP="${1:-192.168.60.90}"
OUT=/home/ws/diag_out
mkdir -p "$OUT"
TS=$(date +%Y%m%d_%H%M%S)
LOG="$OUT/discovery_probe_$TS.txt"

# candidate ROS_DOMAIN_IDs to try (current env first, then common ones)
CAND_DOMAINS="${ROS_DOMAIN_ID:-} 0 42"

exec > >(tee "$LOG") 2>&1

sec() { echo; echo "===================== $* ====================="; }
have() { command -v "$1" >/dev/null 2>&1; }
run() { echo "\$ $*"; timeout 20 "$@" 2>&1; echo "(exit $?)"; }

echo "discovery_probe.sh  $TS"
echo "ONBOARD_IP = $ONBOARD_IP"
echo "log        = $LOG"

sec "0. host / user / date"
run id
run hostname
run uname -a
date

sec "1. ROS environment (as currently exported in this shell)"
env | grep -Ei 'ROS|RMW|FASTRTPS|CYCLONE|DDS' | sort
echo "---"
echo "ROS_DISTRO           = ${ROS_DISTRO:-<unset>}"
echo "ROS_DOMAIN_ID        = ${ROS_DOMAIN_ID:-<unset>}"
echo "ROS_LOCALHOST_ONLY   = ${ROS_LOCALHOST_ONLY:-<unset>}   (must be 0 for networked ROS2)"
echo "RMW_IMPLEMENTATION   = ${RMW_IMPLEMENTATION:-<unset>}    (Humble default: rmw_fastrtps_cpp)"
echo "ROS_STATIC_PEERS     = ${ROS_STATIC_PEERS:-<unset>}"
echo "CYCLONEDDS_URI       = ${CYCLONEDDS_URI:-<unset>}"
echo "FASTRTPS_DEFAULT_PROFILES_FILE = ${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}"

sec "2. network interfaces + addresses"
if have ip; then
  run ip -brief addr
  run ip -4 addr
else
  echo "(no 'ip' — sudo apt install -y iproute2 for full output; using fallbacks)"
  run hostname -I
  for n in /sys/class/net/*; do
    d=$(basename "$n")
    echo "$d  mac=$(cat "$n/address" 2>/dev/null)  carrier=$(cat "$n/carrier" 2>/dev/null)  oper=$(cat "$n/operstate" 2>/dev/null)"
  done
fi

sec "3. routing table"
if have ip; then
  run ip route
  run ip -4 route get "$ONBOARD_IP"
else
  echo "--- /proc/net/route ---"; cat /proc/net/route
fi

sec "4. multicast route (needed for default DDS discovery over a direct link)"
if have ip; then
  ip route | grep -E '224\.0\.0\.0/4' && echo "  -> multicast route present" \
    || echo "  -> NO 224.0.0.0/4 route. If discovery fails, add: sudo ip route add 224.0.0.0/4 dev <iface>"
fi
echo "--- igmp / multicast group membership ---"
cat /proc/net/igmp 2>/dev/null | head -40

sec "5. ARP / neighbour table"
if have ip; then run ip neigh; else echo "--- /proc/net/arp ---"; cat /proc/net/arp; fi

sec "6. reachability to onboard PC ($ONBOARD_IP)"
if have ping; then
  run ping -c 4 -W 2 "$ONBOARD_IP"
else
  echo "(no 'ping' — sudo apt install -y iputils-ping)"
fi
echo "--- TCP reach test to onboard SSH (22) ---"
if timeout 4 bash -c "echo > /dev/tcp/$ONBOARD_IP/22" 2>/dev/null; then
  echo "  OK: TCP 22 open on $ONBOARD_IP (onboard PC reachable)"
else
  echo "  FAIL: cannot open TCP 22 on $ONBOARD_IP (no L3 path? wrong IP? firewall?)"
fi

sec "7. laptop firewall (may need sudo for full detail)"
run ufw status verbose
echo "--- nft ruleset (needs sudo) ---"
run sudo -n nft list ruleset
echo "--- iptables (needs sudo) ---"
run sudo -n iptables -L -n

sec "8. ros2 daemon reset"
run timeout 10 ros2 daemon stop
sleep 1

sec "9. DDS multicast self-test (ROS_LOCALHOST_ONLY=0)"
echo "sender in background for 5s, receiver listens..."
( export ROS_LOCALHOST_ONLY=0; timeout 6 ros2 multicast send >/dev/null 2>&1 & )
export ROS_LOCALHOST_ONLY=0
run timeout 6 ros2 multicast receive

sec "10. discovery attempts across candidate ROS_DOMAIN_IDs"
ARM_RE='joint_states|dynamic_joint_states|joint_trajectory_controller|scaled_joint|controller_manager|/tf|io_and_status|force_torque|ur5|robot_description'
seen=""
for D in $CAND_DOMAINS; do
  [ -z "$D" ] && continue
  case " $seen " in *" $D "*) continue;; esac
  seen="$seen $D"
  echo
  echo "########## ROS_DOMAIN_ID=$D  ROS_LOCALHOST_ONLY=0 ##########"
  export ROS_LOCALHOST_ONLY=0
  export ROS_DOMAIN_ID="$D"
  list=$(timeout 20 ros2 topic list --no-daemon 2>&1)
  n=$(echo "$list" | grep -c '^/')
  echo "topic count: $n"
  echo "$list"
  echo "--- arm-related topics on domain $D ---"
  echo "$list" | grep -E "$ARM_RE" || echo "(none)"
  echo "--- nodes on domain $D ---"
  timeout 20 ros2 node list --no-daemon 2>&1
done

sec "11. ros2 doctor (last domain tried)"
run timeout 30 ros2 doctor --report

sec "DONE"
echo "Log written to: $LOG"
echo
echo "Next: paste the path to Claude, or just say it's done."
