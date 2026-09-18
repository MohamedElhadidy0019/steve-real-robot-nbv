#!/usr/bin/env python3
"""
STEP 3 of the real-robot NBV pipeline (final standalone stage): given the
object pose (step 1, estimate_object_pose.py) and the reachability-filtered
candidate viewpoints (step 2, sample_object_viewpoints.py), plan ONE
best-next-view step per invocation -- SINGLE-STEP AND STATEFUL, meant to be
called in a loop by run_nbv_loop.sh, which does the real send-and-WAIT
between calls. This is deliberately NOT an open-loop "plan the whole tour"
script: the arm's start state for each step must come from a FRESH read of
its ACTUAL current pose (dumped by ros_joint_state_dump.py right before
this runs), never an assumed/planned position -- a real robot can end up
slightly off from what was planned, and errors would compound across a
multi-view tour if each step trusted the previous step's plan instead of
reality.

Progress (which mesh points are seen, which candidates already visited) is
persisted to --state between calls, since each invocation is a fresh
process. State schema: local mesh sample points/normals (frozen at the
FIRST call so "seen" indices stay meaningful across calls), seen/visited
arrays, and a "pending" entry (this call's chosen candidate + its predicted-
visible mask) that is NOT yet merged into "seen" -- see commit_capture.py,
which only commits it after run_nbv_loop.sh confirms the move actually
succeeded. This matters even for the dummy-camera case: if planning
succeeds here but the send later fails (safety stop, controller fault --
all things that have actually happened this project), we must not credit a
view as "seen" that the arm never reached.

No live camera right now (robot off), so the "capture" step (in
commit_capture.py, not here) is a deliberate stand-in: instead of
segmenting a real depth image and backprojecting it, it reuses the EXACT
SAME ray-visibility function this script already uses to SCORE candidates.
In the noiseless limit that's exactly correct (the score already predicts
what a perfect sensor would see from that pose) -- it validates everything
else in this loop (reachability, greedy selection, Cartesian planning,
obstacle avoidance, stopping criteria, real-pose closed-loop chaining)
end-to-end, but does NOT validate real sensor/segmentation accuracy.

Exit codes (run_nbv_loop.sh branches on these):
    0 = planned a step; trajectory + pending capture written, keep looping
    2 = done (coverage target reached, or nothing useful left) -- no
        trajectory written, stop the loop cleanly
    3 = real failure -- no candidate could be planned to at all

This script is pure CuRobo/numpy/torch (rob_env, Python 3.12) -- no rclpy,
matching the project's hard rclpy/py3.10-vs-CuRobo/py3.12 ABI split. It
never sends anything to the robot itself.

Run (inside rob_env, normally called BY run_nbv_loop.sh, not by hand):
    conda activate rob_env
    cd /home/ws/full_pipeline
    python3 step3_nbv_motion_planning.py --state ... --start ... --traj-out ...
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, "/home/ws")  # object_geometry.py, floor_geometry.py
from floor_geometry import DEFAULT_FLOOR_OFFSET, DEFAULT_FLOOR_SIZE, FLOOR_THICKNESS, floor_center_z
from object_geometry import DEFAULT_OBJECT_POSE_PATH, OBJECT_MARGIN_M, load_object_pose, object_cuboid_pose_and_dims

DEFAULT_CACHE_PATH = "/home/ws/nbv_scratch/cereal_box/viewpoint_cache.json"
DEFAULT_START_PATH = "/home/ws/nbv_scratch/current_joint_state.json"
DEFAULT_TRAJ_OUT = "/home/ws/nbv_scratch/cereal_box/nbv_tour_trajectory.json"
DEFAULT_LOG_OUT = "/home/ws/nbv_scratch/cereal_box/nbv_tour_log.json"
CANONICAL = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
XYZW_TO_WXYZ = [3, 0, 1, 2]

# Local box faces, FIXED convention matching BoxIcpTracker's R columns
# (x=thin, y=mid, z=wide) -- box_icp_ros_bridge.py. axis index into
# (thin, mid, wide), sign is which side along that axis.
FACES = [(0, -1), (0, +1), (1, -1), (1, +1), (2, -1), (2, +1)]


# --------------------------------------------------------------------------
# Procedural box mesh (no CAD/trimesh dependency -- the object IS a box,
# dims already known from object_pose.json, and rob_env has no trimesh).
# --------------------------------------------------------------------------
def sample_box_points_with_normals(dims, n_per_face, exclude_face=None):
    """dims = [thin, mid, wide] (local x, y, z half-extents doubled -- same
    convention as BoxIcpTracker.dims). Returns (points (N,3), normals (N,3),
    face_idx (N,)) in the box's own LOCAL frame, centered at origin."""
    half = np.asarray(dims) / 2.0
    pts, nrm, faces = [], [], []
    for fi, (axis, sign) in enumerate(FACES):
        if fi == exclude_face:
            continue
        other = [a for a in range(3) if a != axis]
        u = np.random.uniform(-1, 1, n_per_face) * half[other[0]]
        v = np.random.uniform(-1, 1, n_per_face) * half[other[1]]
        p = np.zeros((n_per_face, 3))
        p[:, other[0]] = u
        p[:, other[1]] = v
        p[:, axis] = sign * half[axis]
        n = np.zeros((n_per_face, 3))
        n[:, axis] = sign
        pts.append(p)
        nrm.append(n)
        faces.append(np.full(n_per_face, fi))
    return np.concatenate(pts), np.concatenate(nrm), np.concatenate(faces)


def box_triangles_local(dims):
    """12 triangles (2 per face), box centered at origin, local frame --
    used only for the self-occlusion ray test."""
    half = np.asarray(dims) / 2.0
    tris = []
    for axis, sign in FACES:
        other = [a for a in range(3) if a != axis]
        corners = []
        for su in (-1, 1):
            for sv in (-1, 1):
                c = np.zeros(3)
                c[other[0]] = su * half[other[0]]
                c[other[1]] = sv * half[other[1]]
                c[axis] = sign * half[axis]
                corners.append(c)
        # corners: (--),(-+),(+-),(++) in (u,v) -- two triangles per face
        tris.append([corners[0], corners[1], corners[2]])
        tris.append([corners[1], corners[3], corners[2]])
    return np.array(tris)  # (12, 3, 3)


def find_bottom_face(q_obj_xyzw):
    """Which local face's outward normal ends up most aligned with -Z_world
    after the object's actual solved rotation -- that face rests on the
    table and can never be seen, so it's excluded from the coverage target
    entirely (matches the sim's own "excluded base" convention, rather than
    chasing an unreachable 100%)."""
    R = Rotation.from_quat(q_obj_xyzw).as_matrix()
    best_fi, best_dot = None, 1.0
    for fi, (axis, sign) in enumerate(FACES):
        local_n = np.zeros(3)
        local_n[axis] = sign
        world_n = R @ local_n
        dot = world_n[2]  # alignment with +Z_world; most negative = most "down"
        if dot < best_dot:
            best_dot, best_fi = dot, fi
    return best_fi


def transform_points(points_local, normals_local, t_obj, q_obj_xyzw):
    R = Rotation.from_quat(q_obj_xyzw)
    points_world = R.apply(points_local) + t_obj[None, :]
    normals_world = R.apply(normals_local)
    return points_world, normals_world


# --------------------------------------------------------------------------
# GPU batched ray-triangle visibility (Moller-Trumbore) -- reimplemented in
# the same spirit as the sim's nbv_core/ray_scoring.py, not copy-pasted
# (different mesh representation here: procedural box, not a loaded mesh).
# For a CONVEX shape like a box, front-facing alone is actually already
# sufficient for correct self-visibility -- the occlusion test below is
# mathematically redundant here, but kept for generality/correctness
# assurance and to match the sim's real algorithm shape.
# --------------------------------------------------------------------------
def ray_triangle_intersect(origins, dirs, tris, eps=1e-7):
    """origins (R,3), dirs (R,3) [not required unit], tris (T,3,3) -> t (R,T)
    with +inf where no hit / behind the ray origin. Pure torch, batched."""
    import torch
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]  # (T,3)
    e1 = (v1 - v0)[None, :, :]  # (1,T,3)
    e2 = (v2 - v0)[None, :, :]
    d = dirs[:, None, :]        # (R,1,3)
    o = origins[:, None, :]     # (R,1,3)

    pvec = torch.cross(d.expand(-1, tris.shape[0], -1), e2.expand(d.shape[0], -1, -1), dim=-1)
    det = (e1 * pvec).sum(-1)  # (R,T)
    inv_det = torch.where(det.abs() > eps, 1.0 / det, torch.zeros_like(det))

    tvec = o - v0[None, :, :]
    u = (tvec * pvec).sum(-1) * inv_det

    qvec = torch.cross(tvec, e1.expand(tvec.shape[0], -1, -1), dim=-1)
    v = (d * qvec).sum(-1) * inv_det

    t = (e2.expand(d.shape[0], -1, -1) * qvec).sum(-1) * inv_det

    hit = (det.abs() > eps) & (u >= 0) & (u <= 1) & (v >= 0) & (u + v <= 1) & (t > eps)
    return torch.where(hit, t, torch.full_like(t, float("inf")))


def compute_visible_mask(cam_pos, points_world, normals_world, tris_world,
                          front_margin, tensor_args):
    """One candidate camera position at a time -- points_world/normals_world
    (N,3) already-transformed mesh samples, tris_world (T,3,3) already-
    transformed mesh triangles. Returns boolean (N,) numpy."""
    import torch
    p = tensor_args.to_device(points_world.astype(np.float32))
    n = tensor_args.to_device(normals_world.astype(np.float32))
    c = tensor_args.to_device(cam_pos.astype(np.float32))
    tris = tensor_args.to_device(tris_world.astype(np.float32))

    to_cam = c[None, :] - p
    dist = to_cam.norm(dim=-1)
    to_cam_unit = to_cam / dist[:, None].clamp_min(1e-9)
    front = (n * to_cam_unit).sum(-1) > front_margin

    # Nudge ray origins slightly off the surface along their own normal --
    # otherwise a point's own triangle can register a spurious self-hit at
    # t~0 from floating-point noise (classic ray-tracing "shadow acne"),
    # which would wrongly mark points as occluded by themselves.
    p_offset = p + n * 1e-4
    t_hits = ray_triangle_intersect(p_offset, to_cam, tris)  # (N,T)
    nearest_hit = t_hits.min(dim=-1).values
    not_occluded = nearest_hit > (1.0 - 1e-3)  # dirs weren't normalized; t=1 reaches the point

    visible = front & not_occluded
    torch.cuda.synchronize()
    return visible.cpu().numpy()


DEFAULT_STATE_PATH = "/home/ws/nbv_scratch/cereal_box/nbv_state.json"


def init_state(object_pose_file, n_per_face):
    obj = load_object_pose(object_pose_file)
    q_obj_xyzw = np.array(obj["q_arm_base_xyzw"])
    dims = obj["dims_m"]
    bottom_face = find_bottom_face(q_obj_xyzw)
    pts_local, nrm_local, _ = sample_box_points_with_normals(
        dims, n_per_face, exclude_face=bottom_face)
    print(f"[init] object dims (thin,mid,wide)={dims}, excluding face {bottom_face} "
          f"(rests on the table), {len(pts_local)} coverage points sampled")
    return {
        "points_local": pts_local.tolist(),
        "normals_local": nrm_local.tolist(),
        "dims": dims,
        "bottom_face_excluded": int(bottom_face),
        "seen": [False] * len(pts_local),
        "visited": [],
        "pending": None,
        "iteration": 0,
    }


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--object-pose-file", default=DEFAULT_OBJECT_POSE_PATH)
    ap.add_argument("--cache-file", default=DEFAULT_CACHE_PATH)
    ap.add_argument("--start", default=DEFAULT_START_PATH)
    ap.add_argument("--state", default=DEFAULT_STATE_PATH)
    ap.add_argument("--traj-out", default=DEFAULT_TRAJ_OUT)
    ap.add_argument("--reset", action="store_true",
                     help="ignore any existing --state file, start a fresh tour")
    ap.add_argument("--n-per-face", type=int, default=40)
    ap.add_argument("--front-margin", type=float, default=0.1)
    ap.add_argument("--coverage-target", type=float, default=0.9)
    ap.add_argument("--min-new-points", type=int, default=3,
                     help="stop once the BEST remaining candidate reveals fewer "
                          "than this many new points")
    ap.add_argument("--no-floor", action="store_true")
    ap.add_argument("--floor-offset", type=float, default=DEFAULT_FLOOR_OFFSET)
    ap.add_argument("--floor-size", type=float, default=DEFAULT_FLOOR_SIZE)
    ap.add_argument("--no-object", action="store_true")
    ap.add_argument("--object-margin", type=float, default=OBJECT_MARGIN_M)
    args = ap.parse_args()

    if (not args.reset) and os.path.exists(args.state):
        with open(args.state) as f:
            state = json.load(f)
        print(f"[state] loaded existing tour state, iteration={state['iteration']}")
    else:
        state = init_state(args.object_pose_file, args.n_per_face)
        print("[state] starting a FRESH tour (no existing state, or --reset)")

    if state["pending"] is not None:
        print("! ERROR: --state has an uncommitted 'pending' entry from a previous "
              "call -- run commit_capture.py (after confirming the move succeeded) "
              "or fix/discard the state file before planning another step.")
        raise SystemExit(3)

    obj = load_object_pose(args.object_pose_file)
    t_obj = np.array(obj["t_arm_base"])
    q_obj_xyzw = np.array(obj["q_arm_base_xyzw"])
    dims = state["dims"]

    pts_local = np.array(state["points_local"])
    nrm_local = np.array(state["normals_local"])
    seen = np.array(state["seen"], dtype=bool)
    visited = set(state["visited"])
    n_points = len(pts_local)

    pts_world, nrm_world = transform_points(pts_local, nrm_local, t_obj, q_obj_xyzw)
    tris_local = box_triangles_local(dims)
    R_obj = Rotation.from_quat(q_obj_xyzw)
    tris_world = R_obj.apply(tris_local.reshape(-1, 3)).reshape(-1, 3, 3) + t_obj[None, None, :]

    with open(args.cache_file) as f:
        cache = json.load(f)
    reachable = np.array(cache["reachable_collision_aware"])
    t_cam_all = np.array(cache["t_candidates_camera"])
    t_tool0_all = np.array(cache["t_tool0_targets"])
    q_tool0_all = np.array(cache["q_tool0_targets_xyzw"])

    with open(args.start) as f:
        start = json.load(f)
    assert start["joint_names"] == CANONICAL
    start_positions = list(start["positions"])
    print(f"[iteration {state['iteration']}] ACTUAL current pose (rad):",
          [f"{v:+.4f}" for v in start_positions])

    # --- score every reachable, not-yet-visited candidate ---
    from curobo.types.base import TensorDeviceType
    tensor_args = TensorDeviceType()

    candidate_idxs = [i for i in np.where(reachable)[0] if i not in visited]
    coverage = seen.sum() / n_points
    if not candidate_idxs:
        print(f"no reachable candidates left -- DONE (coverage={coverage:.1%})")
        raise SystemExit(2)

    scored = []
    for i in candidate_idxs:
        vis = compute_visible_mask(t_cam_all[i], pts_world, nrm_world, tris_world,
                                    args.front_margin, tensor_args)
        new_count = int((vis & ~seen).sum())
        scored.append((new_count, i, vis))
    scored.sort(key=lambda x: -x[0])

    best_new = scored[0][0]
    print(f"best candidate={scored[0][1]}, new_points={best_new}, coverage_so_far={coverage:.1%}")
    if best_new < args.min_new_points or coverage >= args.coverage_target:
        print(f"DONE: best_new={best_new} < min={args.min_new_points} "
              f"or coverage {coverage:.1%} >= target {args.coverage_target:.1%}")
        raise SystemExit(2)

    # --- obstacles (same as every other real-robot CuRobo call) ---
    from curobo.geom.sdf.world import CollisionCheckerType
    from curobo.geom.types import Cuboid, WorldConfig

    cuboids = []
    if not args.no_floor:
        center_z = floor_center_z(args.floor_offset, FLOOR_THICKNESS)
        cuboids.append(Cuboid(name="floor", pose=[0, 0, center_z, 1, 0, 0, 0],
                               dims=[args.floor_size, args.floor_size, FLOOR_THICKNESS]))
    if not args.no_object and os.path.exists(args.object_pose_file):
        obj_pose, obj_dims = object_cuboid_pose_and_dims(args.object_pose_file, args.object_margin)
        cuboids.append(Cuboid(name="object", pose=obj_pose, dims=obj_dims))
    world_cfg = WorldConfig(cuboid=cuboids) if cuboids else None

    from curobo.types.math import Pose
    from curobo.types.robot import JointState
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

    motion_gen_cfg = MotionGenConfig.load_from_robot_config(
        "ur5e.yml", world_cfg, tensor_args,
        collision_checker_type=CollisionCheckerType.PRIMITIVE, interpolation_dt=0.02)
    motion_gen = MotionGen(motion_gen_cfg)
    print("warming up motion_gen (one-time CUDA kernel warmup)...")
    motion_gen.warmup()

    q_start = JointState.from_position(
        tensor_args.to_device([start_positions]), joint_names=CANONICAL)

    planned = None
    for new_count, i, vis in scored:
        if new_count < args.min_new_points:
            break
        q_wxyz = q_tool0_all[i][XYZW_TO_WXYZ]
        goal_pose = Pose(
            tensor_args.to_device([t_tool0_all[i].tolist()]),
            tensor_args.to_device([q_wxyz.tolist()]),
        )
        result = motion_gen.plan_single(q_start, goal_pose, MotionGenPlanConfig(max_attempts=10))
        if bool(result.success.item()):
            planned = (i, new_count, vis, result)
            break
        print(f"  candidate {i} (new_points={new_count}): planning FAILED "
              f"(status: {result.status}) -- trying next-best candidate")

    if planned is None:
        print("! no candidate could be planned to from the current pose -- real failure")
        raise SystemExit(3)

    best_i, best_new, best_vis, result = planned
    traj = result.get_interpolated_plan()
    positions = traj.position.cpu().numpy().tolist()
    dt = float(result.interpolation_dt)
    times = [k * dt for k in range(len(positions))]

    with open(args.traj_out, "w") as f:
        json.dump({"joint_names": CANONICAL, "positions": positions, "time_from_start": times}, f)
    print(f"planned {len(positions)} waypoints, {times[-1]:.2f}s -> {args.traj_out}")

    # NOT committed to "seen"/"visited" yet -- see commit_capture.py. This
    # is the DUMMY CAPTURE prediction; a real camera would instead verify
    # what was actually seen once the arm has genuinely arrived.
    state["pending"] = {"candidate": int(best_i), "new_points": int(best_new),
                         "visible_mask": best_vis.tolist()}
    state["iteration"] += 1
    with open(args.state, "w") as f:
        json.dump(state, f)
    print(f"wrote pending state (candidate {best_i}, {best_new} predicted new points) "
          f"-> {args.state}")


if __name__ == "__main__":
    main()
