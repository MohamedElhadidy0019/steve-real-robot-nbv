#!/usr/bin/env python3
"""
First real script of the FINAL real-robot NBV pipeline: given ONE candidate
viewpoint from sample_object_viewpoints.py's reachability cache, run a real
CuRobo CARTESIAN motion plan (MotionGen.plan_single, not the joint-space
plan_single_js every other real-robot script so far has used) from the
arm's CURRENT pose to that viewpoint's tool0 target, collision-aware against
the same table + object obstacles as the rest of the project.

This is deliberately the first file in a NEW folder (full_pipeline/), not
another addition to the flat /home/ws script pile -- everything it imports
from /home/ws (object_geometry, floor_geometry) is imported, not copied, so
there is exactly one source of truth for that geometry. This script is
meant to become one stage of the eventual full real-robot orchestration
loop; for now it's run by hand, one viewpoint at a time, so its output can
be visually validated in RViz (mock mirror) BEFORE it's ever pointed at the
live arm -- see run instructions below.

Why Cartesian (plan_single) here and not just reusing the cache's own
q_joints_collision_aware as a joint-space goal: the cache's IK solve used
whatever seed the batch IK solver happened to converge to, independent of
where the arm actually is right now. plan_single re-solves from the arm's
REAL current state, which is what the eventual orchestration loop actually
needs (it won't always start from the same place) -- and is the same
capability gap (item 3 in the project's real-robot gap list) this script
exists to close.

Run (inside rob_env):
    conda activate rob_env
    cd /home/ws/full_pipeline
    python3 plan_to_viewpoint.py --list
    python3 plan_to_viewpoint.py --index 42

Then send the result to the MOCK first (never the live robot until you've
watched it in RViz and it looks right):
    conda deactivate
    source /home/ws/steve_env.sh
    python3 /home/ws/ros_send_trajectory.py \
        --traj /home/ws/nbv_scratch/planned_trajectory.json --target mock
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, "/home/ws")  # object_geometry.py, floor_geometry.py live there
from floor_geometry import DEFAULT_FLOOR_OFFSET, DEFAULT_FLOOR_SIZE, FLOOR_THICKNESS, floor_center_z
from object_geometry import DEFAULT_OBJECT_POSE_PATH, OBJECT_MARGIN_M, object_cuboid_pose_and_dims

DEFAULT_CACHE_PATH = "/home/ws/nbv_scratch/cereal_box/viewpoint_cache.json"
DEFAULT_START_PATH = "/home/ws/nbv_scratch/current_joint_state.json"
DEFAULT_OUT_PATH = "/home/ws/nbv_scratch/planned_trajectory.json"
CANONICAL = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
XYZW_TO_WXYZ = [3, 0, 1, 2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache-file", default=DEFAULT_CACHE_PATH)
    ap.add_argument("--index", type=int, default=None,
                     help="candidate index into the cache (matches the Marker "
                          "id shown in RViz's /viewpoint_markers)")
    ap.add_argument("--list", action="store_true",
                     help="print collision-aware-reachable (green) candidate "
                          "indices and exit, no planning")
    ap.add_argument("--start", default=DEFAULT_START_PATH,
                     help="current joint state JSON, written by ros_joint_state_dump.py "
                          "(rclpy, run separately -- this process can't import rclpy)")
    ap.add_argument("--out", default=DEFAULT_OUT_PATH)
    ap.add_argument("--no-floor", action="store_true")
    ap.add_argument("--floor-offset", type=float, default=DEFAULT_FLOOR_OFFSET)
    ap.add_argument("--floor-size", type=float, default=DEFAULT_FLOOR_SIZE)
    ap.add_argument("--no-object", action="store_true")
    ap.add_argument("--object-pose-file", default=DEFAULT_OBJECT_POSE_PATH)
    ap.add_argument("--object-margin", type=float, default=OBJECT_MARGIN_M)
    args = ap.parse_args()

    with open(args.cache_file) as f:
        cache = json.load(f)
    reachable = cache["reachable_collision_aware"]
    green = [i for i, ok in enumerate(reachable) if ok]

    if args.list or args.index is None:
        print(f"{len(green)}/{len(reachable)} candidates are collision-aware reachable (green).")
        print("indices:", green)
        if not args.list:
            print("pass --index <N> to plan to one of these (or any index, with a warning "
                  "if it's not green).")
        return

    i = args.index
    if not reachable[i]:
        print(f"! WARNING: candidate {i} was NOT marked collision-aware reachable in the "
              f"cache -- planning anyway (this call re-solves from scratch and may still "
              f"succeed or fail differently), but don't be surprised if it fails.")

    t_tool0 = np.array(cache["t_tool0_targets"][i])
    q_tool0_xyzw = np.array(cache["q_tool0_targets_xyzw"][i])
    print(f"candidate {i}: camera at {cache['t_candidates_camera'][i]}, "
          f"tool0 target t={t_tool0.tolist()}")

    with open(args.start) as f:
        start = json.load(f)
    assert start["joint_names"] == CANONICAL, "start-state JSON joint order mismatch"
    start_positions = start["positions"]
    print("start pose (rad):", [f"{v:+.4f}" for v in start_positions])

    # --- same table + object obstacles as curobo_plan_real_spike.py / sample_object_viewpoints.py ---
    from curobo.geom.sdf.world import CollisionCheckerType
    from curobo.geom.types import Cuboid, WorldConfig

    cuboids = []
    if not args.no_floor:
        center_z = floor_center_z(args.floor_offset, FLOOR_THICKNESS)
        cuboids.append(Cuboid(
            name="floor",
            pose=[0.0, 0.0, center_z, 1, 0, 0, 0],
            dims=[args.floor_size, args.floor_size, FLOOR_THICKNESS],
        ))
    if not args.no_object and os.path.exists(args.object_pose_file):
        obj_pose, obj_dims = object_cuboid_pose_and_dims(args.object_pose_file, args.object_margin)
        cuboids.append(Cuboid(name="object", pose=obj_pose, dims=obj_dims))
    world_cfg = WorldConfig(cuboid=cuboids) if cuboids else None
    print(f"obstacles: {[c.name for c in cuboids] or 'none'}")

    # --- CuRobo Cartesian motion plan (plan_single, NOT plan_single_js) ---
    import torch
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.robot import JointState
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

    tensor_args = TensorDeviceType()
    motion_gen_cfg = MotionGenConfig.load_from_robot_config(
        "ur5e.yml",
        world_cfg,
        tensor_args,
        collision_checker_type=CollisionCheckerType.PRIMITIVE,
        interpolation_dt=0.02,
    )
    motion_gen = MotionGen(motion_gen_cfg)
    print("warming up motion_gen (first-call CUDA kernel warmup, can take a bit)...")
    motion_gen.warmup()

    q_start = JointState.from_position(
        tensor_args.to_device([start_positions]), joint_names=CANONICAL)

    q_wxyz = q_tool0_xyzw[XYZW_TO_WXYZ]
    goal_pose = Pose(
        tensor_args.to_device([t_tool0.tolist()]),
        tensor_args.to_device([q_wxyz.tolist()]),
    )

    result = motion_gen.plan_single(q_start, goal_pose, MotionGenPlanConfig(max_attempts=10))

    if not bool(result.success.item()):
        print(f"! planning FAILED (status: {result.status})")
        raise SystemExit(1)

    traj = result.get_interpolated_plan()
    positions = traj.position.cpu().numpy().tolist()
    dt = float(result.interpolation_dt)
    times = [k * dt for k in range(len(positions))]

    print(f"planned {len(positions)} waypoints, interpolation_dt={dt:.4f}s, "
          f"total duration={times[-1]:.2f}s")

    out = {
        "joint_names": CANONICAL,
        "positions": positions,
        "time_from_start": times,
    }
    with open(args.out, "w") as f:
        json.dump(out, f)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
