#!/bin/bash
# mirror_real_to_mock.sh -- ONE command: read the REAL robot's live pose
# (domain 74), read wherever the MOCK currently sits (domain 0), plan a
# CuRobo trajectory between them, and send it to the mock so RViz mimics
# the real robot. Re-run any time the real robot moves to refresh the
# mock/RViz snapshot.
#
# Requires the mock bringup already running (launch_mock.sh) and, if you
# want to watch, RViz open (launch_rviz.sh) -- both on ROS_DOMAIN_ID=0
# ROS_LOCALHOST_ONLY=1, same as this script uses internally.
#
# Usage:
#   bash /home/ws/mirror_real_to_mock.sh
#   bash /home/ws/mirror_real_to_mock.sh --slow-factor 8   (extra args forward to ros_send_trajectory.py)

set -o pipefail
SCRATCH=/home/ws/nbv_scratch
mkdir -p "$SCRATCH"

echo "[1/4] reading REAL robot's current pose (domain 74)..."
source /home/ws/steve_env.sh > /dev/null
python3 /home/ws/ros_joint_state_dump.py --prefix ur5e --out "$SCRATCH/current_joint_state.json"
if [ $? -ne 0 ]; then
  echo "! failed to read the real robot's state -- is the bringup up? aborting."
  exit 1
fi

echo "[2/4] reading MOCK's current pose (domain 0)..."
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 \
  python3 /home/ws/ros_joint_state_dump.py --prefix ur5 --out "$SCRATCH/mock_current_joint_state.json"
if [ $? -ne 0 ]; then
  echo "! failed to read the mock's state -- is launch_mock.sh running? aborting."
  exit 1
fi

echo "[3/4] planning via CuRobo (rob_env)..."
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate rob_env
python3 /home/ws/curobo_plan_real_spike.py \
    --start "$SCRATCH/mock_current_joint_state.json" \
    --goal  "$SCRATCH/current_joint_state.json" \
    --out   "$SCRATCH/planned_trajectory.json"
plan_status=$?
conda deactivate
if [ $plan_status -ne 0 ]; then
  echo "! CuRobo planning failed -- aborting."
  exit 1
fi

echo "[4/4] sending planned trajectory to mock (domain 0)..."
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 \
  python3 /home/ws/ros_send_trajectory.py \
    --traj "$SCRATCH/planned_trajectory.json" --target mock "$@"
