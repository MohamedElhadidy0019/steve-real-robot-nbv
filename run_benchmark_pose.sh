#!/bin/bash
# run_benchmark_pose.sh -- move to a saved benchmark pose (from save_pose.py),
# planned via CuRobo, ALWAYS starting from the REAL robot's actual current
# live pose (never an assumed/fixed start) -- so it works reaching each pose
# from wherever the arm currently is. --target mock previews the exact same
# plan safely in the mock/RViz first; --target live executes it on the
# physical arm (same safety gate as ros_send_trajectory.py: robot_mode/
# safety_mode check + typed "go" confirmation).
#
# Usage:
#   bash run_benchmark_pose.sh --pose hardest --target mock
#   bash run_benchmark_pose.sh --pose hardest --target live
#   bash run_benchmark_pose.sh --pose hardest --target live --slow-factor 10
#   bash run_benchmark_pose.sh              # no args: lists available poses

set -o pipefail
SCRATCH=/home/ws/nbv_scratch
POSES_FILE="$SCRATCH/benchmark_poses.json"

POSE=""
TARGET=""
EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --pose) POSE="$2"; shift 2 ;;
    --target) TARGET="$2"; shift 2 ;;
    *) EXTRA_ARGS+=("$1"); shift ;;
  esac
done

list_poses() {
  echo "available poses:"
  python3 -c "
import json
try:
    poses = json.load(open('$POSES_FILE'))
    for p in poses:
        print(' -', p['name'])
except FileNotFoundError:
    print('  (no poses saved yet -- see save_pose.py)')
"
}

if [ -z "$POSE" ] || [ -z "$TARGET" ]; then
  echo "usage: run_benchmark_pose.sh --pose <name> --target mock|live [extra args -> ros_send_trajectory.py]"
  list_poses
  exit 1
fi

if [ "$TARGET" != "mock" ] && [ "$TARGET" != "live" ]; then
  echo "! --target must be 'mock' or 'live'"
  exit 1
fi

echo "[1/3] reading REAL robot's current pose (domain 74)..."
source /home/ws/steve_env.sh > /dev/null
python3 /home/ws/ros_joint_state_dump.py --prefix ur5e --out "$SCRATCH/current_joint_state.json"
if [ $? -ne 0 ]; then
  echo "! failed to read the real robot's state -- is the bringup up? aborting."
  exit 1
fi

echo "[2/3] loading saved pose '$POSE' and planning via CuRobo (rob_env)..."
python3 -c "
import json, sys
try:
    poses = json.load(open('$POSES_FILE'))
except FileNotFoundError:
    print('! no benchmark_poses.json yet -- save some poses first', file=sys.stderr)
    sys.exit(1)
match = [p for p in poses if p['name'] == '$POSE']
if not match:
    print('! no saved pose named \'$POSE\'', file=sys.stderr)
    sys.exit(1)
json.dump({'joint_names': match[0]['joint_names'], 'positions': match[0]['positions']},
          open('$SCRATCH/benchmark_goal.json', 'w'))
"
if [ $? -ne 0 ]; then
  list_poses
  exit 1
fi

source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate rob_env
python3 /home/ws/curobo_plan_real_spike.py \
    --start "$SCRATCH/current_joint_state.json" \
    --goal  "$SCRATCH/benchmark_goal.json" \
    --out   "$SCRATCH/planned_trajectory.json"
plan_status=$?
conda deactivate
if [ $plan_status -ne 0 ]; then
  echo "! CuRobo planning failed -- aborting."
  exit 1
fi

if [ "$TARGET" == "mock" ]; then
  # The trajectory we just planned starts from the REAL robot's pose -- but
  # the mock's ACTUAL current joint state is almost certainly somewhere
  # else, and ros2_control's trajectory controller REJECTS (aborts, silently
  # from the RViz side -- no motion, just a log line) any goal whose first
  # waypoint doesn't closely match where the mock currently is (0.2 rad
  # tolerance). So: snap the mock to the real robot's current pose FIRST
  # (same trick mirror_real_to_mock.sh uses), then send the actual benchmark
  # move -- guarantees the mock is exactly where the trajectory expects.
  echo "[3/4] syncing mock to the real robot's current pose first (avoids a"
  echo "       silent controller-side goal-rejection if the mock is elsewhere)..."
  ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 \
    python3 /home/ws/ros_joint_state_dump.py --prefix ur5 --out "$SCRATCH/mock_current_joint_state.json"
  if [ $? -eq 0 ]; then
    source "$HOME/miniforge3/etc/profile.d/conda.sh"
    conda activate rob_env
    python3 /home/ws/curobo_plan_real_spike.py \
        --start "$SCRATCH/mock_current_joint_state.json" \
        --goal  "$SCRATCH/current_joint_state.json" \
        --out   "$SCRATCH/sync_trajectory.json"
    sync_status=$?
    conda deactivate
    if [ $sync_status -eq 0 ]; then
      ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 \
        python3 /home/ws/ros_send_trajectory.py \
          --traj "$SCRATCH/sync_trajectory.json" --target mock
    else
      echo "! sync plan failed -- sending the benchmark move anyway, it may be rejected"
    fi
  else
    echo "! couldn't read mock's current state (is launch_mock.sh running?) -- "
    echo "  sending the benchmark move anyway, it may be rejected"
  fi

  echo "[4/4] sending the benchmark move to --target mock ..."
  ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 \
    python3 /home/ws/ros_send_trajectory.py \
      --traj "$SCRATCH/planned_trajectory.json" --target mock "${EXTRA_ARGS[@]}"
else
  echo "[3/3] sending the benchmark move to --target live ..."
  python3 /home/ws/ros_send_trajectory.py \
    --traj "$SCRATCH/planned_trajectory.json" --target live "${EXTRA_ARGS[@]}"
fi
