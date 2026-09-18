#!/usr/bin/env bash
# Publishes the solved hand-eye calibration (ur5etool0 -> D415 color optical
# frame) as a static TF transform, so anything on the ROS graph (FoundationPose,
# NBV viewpoint math) can look up the camera's pose in base_link via TF.
#
# Reads /home/ws/nbv_scratch/handeye_result.json (produced by handeye_solve.py).
# Run this alongside the realsense2_camera node; Ctrl+C to stop.

set -e
RESULT_FILE="/home/ws/nbv_scratch/handeye_result.json"
PARENT_FRAME="ur5etool0"
# NOT "d415_calib_color_optical_frame" -- the realsense2_camera_node itself
# already publishes that exact frame name as part of its own internal tree
# (d415_calib_link -> d415_calib_color_frame -> d415_calib_color_optical_frame).
# Two publishers broadcasting conflicting parents for the same child frame
# makes tf2 nondeterministically serve whichever arrived last -- lookups
# through this chain intermittently fail with "two unconnected trees"
# (confirmed live 2026-09-16). Geometrically identical, distinctly named
# frame instead -- nothing else on the graph claims this name.
CHILD_FRAME="d415_calib_color_optical_frame_handeye"

if [ ! -f "$RESULT_FILE" ]; then
  echo "ERROR: $RESULT_FILE not found. Run handeye_solve.py first."
  exit 1
fi

read -r X Y Z QX QY QZ QW <<EOF
$(python3 -c "
import json
with open('$RESULT_FILE') as f:
    r = json.load(f)
t = r['translation_m']
q = r['quaternion_xyzw']
print(t[0], t[1], t[2], q[0], q[1], q[2], q[3])
")
EOF

echo "Publishing static TF: $PARENT_FRAME -> $CHILD_FRAME"
echo "  translation: [$X, $Y, $Z]"
echo "  quaternion (xyzw): [$QX, $QY, $QZ, $QW]"

exec ros2 run tf2_ros static_transform_publisher \
  --x "$X" --y "$Y" --z "$Z" \
  --qx "$QX" --qy "$QY" --qz "$QZ" --qw "$QW" \
  --frame-id "$PARENT_FRAME" --child-frame-id "$CHILD_FRAME"
