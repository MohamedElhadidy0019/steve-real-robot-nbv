#!/bin/bash
# Verify the UR5 arm joint frames are geometrically correct.
# Run with launch_mock.sh + RViz up.
source /home/ws/install/setup.bash 2>/dev/null || source install/setup.bash

OUT=/home/ws/diag_out
mkdir -p "$OUT"
F="$OUT/arm_check.txt"
: > "$F"
run() { timeout -k 2 "$@"; [ $? -eq 124 ] && echo "*** TIMED OUT ***"; return 0; }

echo "==================== current joint state ====================" >> "$F"
run 6 ros2 topic echo --no-daemon --once --qos-reliability best_effort /joint_states >> "$F" 2>&1

echo -e "\n==================== consecutive link transforms (child in parent frame) ====================" >> "$F"
pairs=(
  "ur5base_link         ur5shoulder_link"
  "ur5shoulder_link     ur5upper_arm_link"
  "ur5upper_arm_link    ur5forearm_link"
  "ur5forearm_link      ur5wrist_1_link"
  "ur5wrist_1_link      ur5wrist_2_link"
  "ur5wrist_2_link      ur5wrist_3_link"
  "ur5wrist_3_link      ur5flange"
  "ur5flange            ur5tool0"
)
for p in "${pairs[@]}"; do
  echo -e "\n----- $p -----" >> "$F"
  run 4 ros2 run tf2_ros tf2_echo $p >> "$F" 2>&1
done

echo -e "\n==================== every arm link in base_link frame ====================" >> "$F"
for l in ur5base_link ur5shoulder_link ur5upper_arm_link ur5forearm_link ur5wrist_1_link ur5wrist_2_link ur5wrist_3_link ur5flange ur5tool0; do
  echo -e "\n----- base_link -> $l -----" >> "$F"
  run 4 ros2 run tf2_ros tf2_echo base_link $l >> "$F" 2>&1
done

echo -e "\n==================== URDF joint origins (ground truth) ====================" >> "$F"
run 12 ros2 param get --no-daemon /robot_state_publisher robot_description 2>/dev/null \
  | python3 -c "
import sys,re
x=sys.stdin.read()
for m in re.finditer(r'<joint name=\"(ur5[^\"]+)\" type=\"([^\"]+)\">(.*?)</joint>', x, re.S):
    name,typ,body=m.groups()
    o=re.search(r'<origin([^/]*)/>',body)
    ax=re.search(r'<axis xyz=\"([^\"]+)\"',body)
    print(f'{name:28s} {typ:9s} origin[{o.group(1).strip() if o else \"?\"}]  axis[{ax.group(1) if ax else \"-\"}]')
" >> "$F" 2>&1

echo "DONE -> $F"
wc -l "$F"
