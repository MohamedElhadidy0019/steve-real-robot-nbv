#!/bin/bash
# run_nbv_loop.sh -- the REAL closed-loop NBV tour runner. Same env-mixing
# pattern as mirror_real_to_mock.sh: each iteration crosses between rclpy
# (system Python, to query/command the robot) and rob_env (CuRobo planning)
# as separate process calls, since the two can never share a Python process.
#
# Each iteration: dump the ARM'S ACTUAL CURRENT pose -> plan ONE best-next-
# view step from that real pose (rob_env) -> send it and WAIT for real
# confirmed completion -> only THEN commit that view as "seen" -> repeat.
# Never assumes a planned move succeeded -- if the send fails for any
# reason (safety stop, controller fault), the loop stops WITHOUT crediting
# that view, rather than silently continuing from a wrong assumed pose.
#
# Usage:
#   bash run_nbv_loop.sh                 # mock, fresh tour
#   bash run_nbv_loop.sh --target live    # real robot (needs 'go' each move)
#   bash run_nbv_loop.sh --resume         # continue an existing tour's state
#
# Requires (for --target mock): launch_mock.sh + launch_rviz.sh already
# running, same as every other mock-targeted script in this project.
# Requires (for --target live): source /home/ws/steve_env.sh in THIS shell
# beforehand is NOT needed -- this script sets its own env per command --
# but the physical arm must actually be reachable (robot_mode=RUNNING,
# External Control running on the pendant).

set -o pipefail

TARGET=mock
RESET_FLAG="--reset"
MAX_ITERATIONS=15
EXTRA_SEND_ARGS=()

while [ $# -gt 0 ]; do
  case "$1" in
    --target) TARGET="$2"; shift 2 ;;
    --resume) RESET_FLAG=""; shift ;;
    --max-iterations) MAX_ITERATIONS="$2"; shift 2 ;;
    *) EXTRA_SEND_ARGS+=("$1"); shift ;;
  esac
done

if [ "$TARGET" = "mock" ]; then
  DUMP_ENV=(ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1)
  PREFIX=ur5
elif [ "$TARGET" = "live" ]; then
  DUMP_ENV=()  # steve_env.sh (domain 74) sourced explicitly below instead
  PREFIX=ur5e
else
  echo "! --target must be 'mock' or 'live'"
  exit 1
fi

SCRATCH=/home/ws/nbv_scratch/cereal_box
mkdir -p "$SCRATCH"
START="$SCRATCH/current_joint_state.json"
TRAJ="$SCRATCH/nbv_step_trajectory.json"
STATE="$SCRATCH/nbv_state.json"

if [ "$TARGET" = "live" ]; then
  source /home/ws/steve_env.sh > /dev/null
fi

for i in $(seq 1 "$MAX_ITERATIONS"); do
  echo "=== iteration $i (target=$TARGET) ==="

  echo "[1/3] reading ACTUAL current pose..."
  env "${DUMP_ENV[@]}" python3 /home/ws/ros_joint_state_dump.py --prefix "$PREFIX" --out "$START"
  if [ $? -ne 0 ]; then
    echo "! failed to read current pose -- is the bringup up? aborting."
    exit 1
  fi

  echo "[2/3] planning one best-next-view step (rob_env)..."
  source "$HOME/miniforge3/etc/profile.d/conda.sh"
  conda activate rob_env
  python3 /home/ws/full_pipeline/step3_nbv_motion_planning.py \
      --state "$STATE" --start "$START" --traj-out "$TRAJ" $RESET_FLAG
  plan_status=$?
  conda deactivate
  RESET_FLAG=""  # only the very first iteration may reset

  if [ $plan_status -eq 2 ]; then
    echo "NBV tour complete -- coverage target reached (or nothing useful left)."
    exit 0
  elif [ $plan_status -ne 0 ]; then
    echo "! planning failed (exit $plan_status) -- aborting."
    exit 1
  fi

  echo "[3/3] sending step and waiting for REAL confirmed completion..."
  env "${DUMP_ENV[@]}" python3 /home/ws/ros_send_trajectory.py \
      --traj "$TRAJ" --target "$TARGET" "${EXTRA_SEND_ARGS[@]}"
  send_status=$?

  if [ $send_status -ne 0 ]; then
    echo "! send FAILED or was refused -- NOT committing this view as seen. "
    echo "  Tour state left as-is; investigate and re-run (without --resume "
    echo "  reset behavior; this run already didn't pass --reset again) once fixed."
    exit 1
  fi

  python3 /home/ws/full_pipeline/commit_capture.py --state "$STATE"
done

echo "! hit --max-iterations ($MAX_ITERATIONS) without the loop reporting done -- stopping."
exit 1
