#!/bin/bash
# Real-robot RViz arm-position diagnostics (v3 - hard timeout on EVERYTHING).
# Run INSIDE the devcontainer, AFTER: bash clean.sh  ->  bash launch_mock.sh (fresh terminal, wait ~15s)

source /home/ws/install/setup.bash 2>/dev/null || source install/setup.bash

OUT=/home/ws/diag_out
mkdir -p "$OUT"
cd "$OUT" || exit 1

# helper: run with a hard timeout, never let anything hang the script
run() { timeout -k 2 "$@"; local rc=$?; [ $rc -eq 124 ] && echo "*** TIMED OUT (hung) ***"; return 0; }

ros2 daemon stop 2>/dev/null; sleep 1; ros2 daemon start 2>/dev/null; sleep 2

echo "=== 0. process check (want exactly ONE ros2_control_node) ===" > 00_procs.txt
ps aux | grep -E 'ros2_control_node|robot_state_publisher|spawner|controller_manager|rviz' | grep -v grep >> 00_procs.txt 2>&1

echo "=== 1. /joint_states (best_effort, 6s) ===" > 01_joint_states.txt
run 8 ros2 topic echo --no-daemon --qos-reliability best_effort /joint_states >> 01_joint_states.txt 2>&1

echo "=== 2. ros2 control (10s each) ===" > 02_controllers.txt
echo "--- list_controllers ---" >> 02_controllers.txt
run 10 ros2 control list_controllers -v >> 02_controllers.txt 2>&1
echo -e "\n--- list_hardware_components ---" >> 02_controllers.txt
run 10 ros2 control list_hardware_components -v >> 02_controllers.txt 2>&1
echo -e "\n--- list_hardware_interfaces ---" >> 02_controllers.txt
run 10 ros2 control list_hardware_interfaces >> 02_controllers.txt 2>&1

echo "=== 3. TF (4s each) ===" > 03_tf.txt
for pair in "base_link ur5tool0" "base_link ur5base_link" "ur5base_link ur5shoulder_link"; do
  echo -e "\n--- tf2_echo $pair ---" >> 03_tf.txt
  run 5 ros2 run tf2_ros tf2_echo $pair >> 03_tf.txt 2>&1
done
echo -e "\n--- view_frames ---" >> 03_tf.txt
run 12 ros2 run tf2_tools view_frames -o "$OUT/frames" >> 03_tf.txt 2>&1

echo "=== 4. nodes ===" > 04_nodes.txt
run 15 ros2 node list --no-daemon >> 04_nodes.txt 2>&1

echo "=== 5. topics ===" > 05_topics.txt
run 15 ros2 topic list -t --no-daemon >> 05_topics.txt 2>&1

echo "=== 6. RSP robot_description: joint tags ===" > 06_robot_description.txt
run 12 ros2 param get --no-daemon /robot_state_publisher robot_description 2>/dev/null \
  | grep -oE '<joint name="[^"]*" type="[^"]*"' >> 06_robot_description.txt 2>&1

echo "=== 7. controller_manager param list (10s) ===" > 07_cm.txt
run 10 ros2 param list --no-daemon /controller_manager >> 07_cm.txt 2>&1

echo "=== 8. controller state topics (4s each) ===" > 08_ctrl_state.txt
for c in joint_trajectory_controller scaled_joint_trajectory_controller joint_state_broadcaster; do
  echo -e "\n--- /$c/controller_state ---" >> 08_ctrl_state.txt
  run 5 ros2 topic echo --no-daemon --once /$c/controller_state >> 08_ctrl_state.txt 2>&1
done

echo "=== 9. ros2 doctor ===" > 09_doctor.txt
run 30 ros2 doctor --report >> 09_doctor.txt 2>&1

echo
echo "DONE -> $OUT"
ls -la "$OUT"
