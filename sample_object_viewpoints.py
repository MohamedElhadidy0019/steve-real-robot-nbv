#!/usr/bin/env python3
"""
Stage A (reachability cache) for the real robot: sample candidate CAMERA
viewpoints in a shell around the already-estimated object pose
(estimate_object_pose.py's output), then batch-check which are actually
IK-reachable via CuRobo -- ported from the sim's nbv_core/reachability.py,
but real-robot-native, not a direct call into the sim code. Two deliberate
differences from the sim version:

1. No world_poses_to_base_link_frame() step. estimate_object_pose.py already
   writes the object's pose directly in the arm's OWN base_link frame
   ({prefix}base_link) -- there's no separate "world frame" on the real
   robot the way there was a PyBullet world origin in sim. Candidates are
   sampled directly in base_link.

2. No camera end-effector in CuRobo's config. The sim's URDF has a
   dummy_camera_link CuRobo can target directly. The real ur5e.yml's ee_link
   is tool0 -- there is no camera in the kinematic chain, and per project
   decision we're not adding one to the URDF. So every sampled CAMERA pose
   is composed through the INVERSE of the solved hand-eye transform
   (handeye_result.json, "camera pose expressed in ur5etool0 frame") to get
   the TOOL0 target CuRobo's IKSolver actually needs.

Also note the look-at convention here is DELIBERATELY different from the
sim's camera_lookat_quaternion_xyzw: the sim's camera uses a Y-up/X-left
convention (whatever PyBullet's virtual camera wanted). The real D415's
color_optical_frame is a standard ROS/OpenCV optical frame (+Z forward,
+X right, +Y down) -- copying the sim's function unchanged would silently
roll every candidate 180 degrees about its own viewing axis relative to how
the hand-eye calibration was actually solved. Re-derived below for the
correct convention.

Runs the IK reachability check TWICE, same candidates both times:
  - "kinematic" pass: world_model=None, self-collision off -- matches the
    sim's Stage A exactly, answers "is this reachable at all, ignoring the
    table/object." Kept because it's a useful sanity signal (near-0%
    kinematic reachability would mean the sampling shell itself is wrong,
    independent of any obstacle-model correctness).
  - "collision_aware" pass: same table + object Cuboid obstacles already
    proven in curobo_plan_real_spike.py, self-collision on -- answers
    "reachable AND collision-free," the number that actually matters for
    picking a real viewpoint to execute.
A candidate reachable kinematically but not collision-aware means the ONLY
IK solutions CuRobo found for it all clip the table or the object.

Run (inside rob_env):
    conda activate rob_env
    python3 sample_object_viewpoints.py
"""
import argparse
import json
import os

import numpy as np
from scipy.spatial.transform import Rotation

from floor_geometry import DEFAULT_FLOOR_OFFSET, DEFAULT_FLOOR_SIZE, FLOOR_THICKNESS, floor_center_z
from object_geometry import DEFAULT_OBJECT_POSE_PATH, OBJECT_MARGIN_M, load_object_pose, object_cuboid_pose_and_dims

DEFAULT_HANDEYE_PATH = "/home/ws/nbv_scratch/handeye_result.json"
DEFAULT_OUT_PATH = "/home/ws/nbv_scratch/cereal_box/viewpoint_cache.json"
CANONICAL = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def camera_lookat_quaternion_xyzw(eye: np.ndarray, target: np.ndarray) -> np.ndarray:
    """ROS/OpenCV optical-frame convention: +Z forward, +X right, +Y down --
    matches the D415 color_optical_frame the hand-eye calibration was solved
    against. NOT the sim's Y-up convention (see module docstring)."""
    z_axis = target - eye
    z_axis = z_axis / np.linalg.norm(z_axis)
    world_up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(z_axis, world_up)) > 0.99:
        world_up = np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(z_axis, world_up)   # "right"
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)     # "down"
    R = np.column_stack([x_axis, y_axis, z_axis])
    return Rotation.from_matrix(R).as_quat()  # xyzw


def sample_candidate_camera_poses(
    t_obj: np.ndarray, r_min: float, r_max: float, n_radius: int,
    n_theta: int, phi_min_deg: float, phi_max_deg: float, n_phi: int,
):
    """Hemisphere-shell candidates around t_obj, directly in base_link frame
    (spherical coords: radius r, azimuth theta full 2pi, elevation phi above
    horizontal). Full 2pi azimuth sampled deliberately -- the IK reachability
    filter naturally rejects the unreachable far side, same reasoning as sim."""
    radii = np.linspace(r_min, r_max, n_radius)
    thetas = np.linspace(0.0, 2 * np.pi, n_theta, endpoint=False)
    phis = np.radians(np.linspace(phi_min_deg, phi_max_deg, n_phi))

    r_grid, theta_grid, phi_grid = np.meshgrid(radii, thetas, phis, indexing="ij")
    r_grid, theta_grid, phi_grid = r_grid.ravel(), theta_grid.ravel(), phi_grid.ravel()

    offsets = np.stack([
        r_grid * np.cos(phi_grid) * np.cos(theta_grid),
        r_grid * np.cos(phi_grid) * np.sin(theta_grid),
        r_grid * np.sin(phi_grid),
    ], axis=-1)
    t_candidates = t_obj[None, :] + offsets
    q_candidates_xyzw = np.stack(
        [camera_lookat_quaternion_xyzw(t, t_obj) for t in t_candidates], axis=0)
    return t_candidates, q_candidates_xyzw


def compose_camera_to_tool0(t_cam, q_cam_xyzw, t_tool0_cam, q_tool0_cam_xyzw):
    """
    Given a desired CAMERA pose in base_link frame, and the calibrated
    tool0->camera transform (handeye_result.json: 'camera pose expressed in
    ur5etool0 frame', i.e. T_tool0_camera), return the TOOL0 pose in
    base_link frame CuRobo's stock ee_link=tool0 IK solver actually needs:

        T_base_camera = T_base_tool0 * T_tool0_camera
        => T_base_tool0 = T_base_camera * inv(T_tool0_camera)
    """
    R_base_cam = Rotation.from_quat(q_cam_xyzw)
    R_tool0_cam = Rotation.from_quat(q_tool0_cam_xyzw)

    R_base_tool0 = R_base_cam * R_tool0_cam.inv()
    t_base_tool0 = t_cam - R_base_tool0.apply(t_tool0_cam)
    return t_base_tool0, R_base_tool0.as_quat()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--object-pose-file", default=DEFAULT_OBJECT_POSE_PATH)
    ap.add_argument("--handeye-file", default=DEFAULT_HANDEYE_PATH)
    ap.add_argument("--out", default=DEFAULT_OUT_PATH)
    ap.add_argument("--r-min", type=float, default=0.25,
                     help="closest camera standoff distance from object center, m")
    ap.add_argument("--r-max", type=float, default=0.40,
                     help="farthest camera standoff distance from object center, m")
    ap.add_argument("--n-radius", type=int, default=3)
    ap.add_argument("--n-theta", type=int, default=24, help="azimuth samples, full 2pi")
    ap.add_argument("--phi-min-deg", type=float, default=10.0)
    ap.add_argument("--phi-max-deg", type=float, default=75.0)
    ap.add_argument("--n-phi", type=int, default=5)
    ap.add_argument("--no-floor", action="store_true",
                     help="exclude the table obstacle from the collision-aware pass")
    ap.add_argument("--floor-offset", type=float, default=DEFAULT_FLOOR_OFFSET)
    ap.add_argument("--floor-size", type=float, default=DEFAULT_FLOOR_SIZE)
    ap.add_argument("--no-object", action="store_true",
                     help="exclude the object obstacle from the collision-aware pass")
    ap.add_argument("--object-margin", type=float, default=OBJECT_MARGIN_M)
    args = ap.parse_args()

    obj = load_object_pose(args.object_pose_file)
    t_obj = np.array(obj["t_arm_base"])
    print(f"object center (base_link): {t_obj.tolist()}")

    with open(args.handeye_file) as f:
        he = json.load(f)
    t_tool0_cam = np.array(he["translation_m"])
    q_tool0_cam_xyzw = np.array(he["quaternion_xyzw"])

    t_cand, q_cand_xyzw = sample_candidate_camera_poses(
        t_obj, args.r_min, args.r_max, args.n_radius, args.n_theta,
        args.phi_min_deg, args.phi_max_deg, args.n_phi)
    n = len(t_cand)
    print(f"sampled {n} candidate camera viewpoints "
          f"(r=[{args.r_min},{args.r_max}]m, {args.n_theta} azimuth x {args.n_phi} elevation)")

    t_tool0_targets = np.zeros_like(t_cand)
    q_tool0_targets_xyzw = np.zeros_like(q_cand_xyzw)
    for i in range(n):
        t_t, q_t = compose_camera_to_tool0(
            t_cand[i], q_cand_xyzw[i], t_tool0_cam, q_tool0_cam_xyzw)
        t_tool0_targets[i] = t_t
        q_tool0_targets_xyzw[i] = q_t

    # --- build the same table + object obstacles curobo_plan_real_spike.py uses ---
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
    print(f"collision-aware pass obstacles: {[c.name for c in cuboids] or 'none'}")

    # --- CuRobo IK batch check (ee_link=tool0, matches every other real-robot
    # CuRobo call in this project -- stock ur5e.yml, no gripper, no camera link).
    # Two passes over the SAME candidates -- see module docstring. ---
    import torch
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

    tensor_args = TensorDeviceType()
    XYZW_TO_WXYZ = [3, 0, 1, 2]
    q_wxyz = q_tool0_targets_xyzw[:, XYZW_TO_WXYZ]
    position = tensor_args.to_device(t_tool0_targets.astype(np.float32))
    quaternion = tensor_args.to_device(np.ascontiguousarray(q_wxyz.astype(np.float32)))
    goal = Pose(position, quaternion)

    def run_ik(world_model, self_collision):
        cfg = IKSolverConfig.load_from_robot_config(
            "ur5e.yml",
            world_model,
            rotation_threshold=0.05,
            position_threshold=0.005,
            num_seeds=20,
            self_collision_check=self_collision,
            self_collision_opt=self_collision,
            collision_checker_type=CollisionCheckerType.PRIMITIVE,
            tensor_args=tensor_args,
            use_cuda_graph=True,
        )
        solver = IKSolver(cfg)
        result = solver.solve_batch(goal)
        torch.cuda.synchronize()
        reach = result.success.squeeze(-1).cpu().numpy().astype(bool)
        joints = result.solution.squeeze(1).cpu().numpy().astype(np.float32)
        joints[~reach] = 0.0
        return reach, joints

    reachable_kinematic, q_joints_kinematic = run_ik(None, self_collision=False)
    reachable_collision, q_joints_collision = run_ik(world_cfg, self_collision=True)

    only_kinematic = reachable_kinematic & ~reachable_collision
    print(f"reachable (kinematic only, ignores table/object): {int(reachable_kinematic.sum())}/{n}")
    print(f"reachable (collision-aware, table+object counted): {int(reachable_collision.sum())}/{n}")
    print(f"  of which blocked ONLY by the environment model (kinematically fine, "
          f"but every IK solution found clips the table/object): {int(only_kinematic.sum())}")

    out = {
        "frame_id": obj["frame_id"],
        "object_t": t_obj.tolist(),
        "t_candidates_camera": t_cand.tolist(),
        "q_candidates_camera_xyzw": q_cand_xyzw.tolist(),
        "t_tool0_targets": t_tool0_targets.tolist(),
        "q_tool0_targets_xyzw": q_tool0_targets_xyzw.tolist(),
        "reachable_kinematic": reachable_kinematic.tolist(),
        "reachable_collision_aware": reachable_collision.tolist(),
        "q_joints_kinematic": q_joints_kinematic.tolist(),
        "q_joints_collision_aware": q_joints_collision.tolist(),
        "joint_names": CANONICAL,
        "params": vars(args),
    }
    with open(args.out, "w") as f:
        json.dump(out, f)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
