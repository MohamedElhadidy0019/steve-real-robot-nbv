"""
Shared object-obstacle geometry -- single source of truth for both
curobo_plan_real_spike.py (rob_env, Python 3.12, uses this for planning) and
publish_object_marker.py (system Python 3.10, rclpy, uses this to visualize
the same box in RViz). Same pairing/reasoning as floor_geometry.py /
publish_floor_marker.py -- deliberately dependency-free (stdlib only, JSON
not numpy) so both Python versions can import it unchanged.

Reads the one-shot pose estimate written by estimate_object_pose.py
(default DEFAULT_OBJECT_POSE_PATH) -- the inspection object's pose, in the
ARM's OWN base_link frame ("{prefix}base_link", e.g. ur5ebase_link on the
live robot) -- the SAME frame floor_geometry.py's table cuboid already uses,
because that's the frame CuRobo actually plans in (its stock ur5e.yml never
models the mobile-base mount at all). Per the user's explicit guarantee, the
object does not move during a run, so this pose is read ONCE per run, not
re-estimated per plan.
"""
import json

DEFAULT_OBJECT_POSE_PATH = "/home/ws/nbv_scratch/cereal_box/object_pose.json"

# Padding added to every dimension before the estimated box becomes a CuRobo
# collision obstacle -- absorbs the one-shot pose estimate's own noise (ICP
# position spread was a few mm in earlier synthetic validation, worst case
# ~15mm) plus the box dims themselves being a rough tape measurement, not
# calibrated. This is a hard geometric margin (the modeled obstacle is
# literally bigger than the measured box), not CuRobo's own soft
# collision-cost margin -- the "don't hit the object" guarantee shouldn't
# depend on how well that cost knob compensates for an inaccurate world
# model.
OBJECT_MARGIN_M = 0.02


def load_object_pose(path=DEFAULT_OBJECT_POSE_PATH):
    with open(path) as f:
        return json.load(f)


def object_cuboid_pose_and_dims(path=DEFAULT_OBJECT_POSE_PATH, margin=OBJECT_MARGIN_M):
    """(pose, dims) ready for curobo.geom.types.Cuboid(pose=pose, dims=dims):
    pose = [x, y, z, qw, qx, qy, qz] (CuRobo's own quaternion order, note NOT
    the xyzw order the file itself is stored in -- ROS/TF convention on disk,
    converted here at the one place that needs CuRobo's convention). dims are
    padded by `margin` on every side vs. the raw estimate."""
    data = load_object_pose(path)
    x, y, z = data["t_arm_base"]
    qx, qy, qz, qw = data["q_arm_base_xyzw"]
    pose = [x, y, z, qw, qx, qy, qz]
    dims = [d + 2 * margin for d in data["dims_m"]]
    return pose, dims
