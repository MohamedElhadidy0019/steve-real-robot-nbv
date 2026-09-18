#!/usr/bin/env python3
"""
First real-robot CuRobo validation spike: plan a small joint-space motion for
Steve's actual UR5e using CuRobo's own stock ur5e.yml/ur5e.urdf (no gripper --
a better fit than the sim's Robotiq config, since the real arm has none).

Deliberately joint-space (plan_single_js), not Cartesian: no IK/orientation
ambiguity to reason about for this first test, just "does the plan+execute
path work end to end". No collision world (world_model=None) -- self-collision
only, matching the agreed no-environment-model first step (test target chosen
as a small, conservative offset from the arm's *actual current* pose, well
clear of anything).

CollisionCheckerType.PRIMITIVE is required, not the default MESH checker --
MESH pulls in NVIDIA warp's torch interop which crashes
(AttributeError: module 'warp' has no attribute 'torch') in this env; already
hit and solved once in the sim pipeline (see project memory), no reason to
rediscover it here.

Reads the current-pose JSON written by ros_joint_state_dump.py (can't read
/joint_states directly -- this process is rob_env, Python 3.12, no rclpy).
Writes a planned-trajectory JSON for ros_send_trajectory.py to execute.

Run (inside rob_env):
    conda activate rob_env
    python3 curobo_plan_real_spike.py \
        --start /home/ws/nbv_scratch/current_joint_state.json \
        --out   /home/ws/nbv_scratch/planned_trajectory.json \
        --offset 0.15 0 0 0 0 0
"""
import argparse
import json
import os

import torch
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import Cuboid, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

from floor_geometry import DEFAULT_FLOOR_OFFSET, DEFAULT_FLOOR_SIZE, FLOOR_THICKNESS, floor_center_z
from object_geometry import DEFAULT_OBJECT_POSE_PATH, OBJECT_MARGIN_M, object_cuboid_pose_and_dims

CANONICAL = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", default="/home/ws/nbv_scratch/current_joint_state.json")
    ap.add_argument("--out", default="/home/ws/nbv_scratch/planned_trajectory.json")
    ap.add_argument("--offset", type=float, nargs=6, default=None,
                     help="rad offset added to the current pose per joint, "
                          "in [pan, lift, elbow, wrist_1, wrist_2, wrist_3] order")
    ap.add_argument("--goal", default=None,
                     help="path to a joint-state JSON (same format as --start) to use "
                          "as an absolute goal instead of --offset -- e.g. drive the mock "
                          "to match the real robot's actual live pose")
    ap.add_argument("--no-floor", action="store_true",
                     help="disable the floor/table collision obstacle (self-collision only, "
                          "matches the very first spike before any environment model existed)")
    ap.add_argument("--floor-offset", type=float, default=DEFAULT_FLOOR_OFFSET,
                     help=f"meters from the arm's base_link DOWN to the modeled floor "
                          f"surface (default {DEFAULT_FLOOR_OFFSET:.3f}, an xacro-derived "
                          f"ESTIMATE with a safety margin baked in -- override once you have "
                          f"a real tape-measure reading)")
    ap.add_argument("--floor-size", type=float, default=DEFAULT_FLOOR_SIZE,
                     help=f"floor/table footprint, meters square, arm-centered "
                          f"(default {DEFAULT_FLOOR_SIZE})")
    ap.add_argument("--no-object", action="store_true",
                     help="disable the inspection-object collision obstacle, even if "
                          "estimate_object_pose.py's output file exists")
    ap.add_argument("--object-pose-file", default=DEFAULT_OBJECT_POSE_PATH,
                     help="pose written by estimate_object_pose.py -- see object_geometry.py")
    ap.add_argument("--object-margin", type=float, default=OBJECT_MARGIN_M,
                     help=f"padding (m) added to every side of the estimated object dims "
                          f"before it becomes a collision obstacle (default {OBJECT_MARGIN_M})")
    args = ap.parse_args()

    with open(args.start) as f:
        start = json.load(f)
    assert start["joint_names"] == CANONICAL, "start-state JSON joint order mismatch"
    start_positions = start["positions"]

    if args.goal:
        with open(args.goal) as f:
            goal = json.load(f)
        assert goal["joint_names"] == CANONICAL, "goal-state JSON joint order mismatch"
        goal_positions = goal["positions"]
    else:
        offset = args.offset or [0.15, 0.0, 0.0, 0.0, 0.0, 0.0]
        goal_positions = [p + o for p, o in zip(start_positions, offset)]

    print("start pose (rad):  ", [f"{v:+.4f}" for v in start_positions])
    print("goal pose  (rad):  ", [f"{v:+.4f}" for v in goal_positions])

    tensor_args = TensorDeviceType()

    cuboids = []
    if not args.no_floor:
        center_z = floor_center_z(args.floor_offset, FLOOR_THICKNESS)
        cuboids.append(Cuboid(
            name="floor",
            pose=[0.0, 0.0, center_z, 1, 0, 0, 0],  # [x,y,z,qw,qx,qy,qz]
            dims=[args.floor_size, args.floor_size, FLOOR_THICKNESS],
        ))
        print(f"floor obstacle: {args.floor_size}x{args.floor_size}m, top surface at "
              f"z={-args.floor_offset:.3f}m relative to arm base_link "
              f"(offset={args.floor_offset:.3f}m -- xacro-derived ESTIMATE, verify with a "
              f"real measurement; pass --no-floor to disable)")
    else:
        print("floor obstacle DISABLED (--no-floor) -- self-collision only")

    if args.no_object:
        print("object obstacle DISABLED (--no-object)")
    elif os.path.exists(args.object_pose_file):
        obj_pose, obj_dims = object_cuboid_pose_and_dims(args.object_pose_file, args.object_margin)
        cuboids.append(Cuboid(name="object", pose=obj_pose, dims=obj_dims))
        print(f"object obstacle: loaded from {args.object_pose_file}, "
              f"padded dims={[round(d, 3) for d in obj_dims]}m (+{args.object_margin * 1000:.0f}mm "
              f"margin), pose(xyz)={[round(v, 3) for v in obj_pose[:3]]}")
    else:
        print(f"! WARNING: object obstacle NOT included -- {args.object_pose_file} does not "
              f"exist yet (run estimate_object_pose.py first). Planning WITHOUT object "
              f"collision protection.")

    world_cfg = WorldConfig(cuboid=cuboids) if cuboids else None

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
    q_goal = JointState.from_position(
        tensor_args.to_device([goal_positions]), joint_names=CANONICAL)

    result = motion_gen.plan_single_js(q_start, q_goal, MotionGenPlanConfig(max_attempts=5))

    if not bool(result.success.item()):
        print(f"! planning FAILED (status: {result.status})")
        raise SystemExit(1)

    traj = result.get_interpolated_plan()
    positions = traj.position.cpu().numpy().tolist()
    dt = float(result.interpolation_dt)
    times = [i * dt for i in range(len(positions))]

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
