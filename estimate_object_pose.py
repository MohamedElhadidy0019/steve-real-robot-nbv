#!/usr/bin/env python3
"""
ONE-SHOT object pose estimate -- run once at the very start of an NBV run,
not a continuous tracker. Per the user's explicit guarantees: the object is
visible in the first frame, and it does not move for the rest of the run --
so unlike box_icp_ros_bridge.py (which re-estimates every frame forever,
built for a "might move" world this project doesn't have), this script
estimates the pose a handful of times, checks the estimates agree, averages
them, writes ONE result, and exits.

Pipeline per attempt (same math as box_icp_ros_bridge.py, reusing its
BoxIcpTracker unchanged -- pure geometry, no rclpy dependency):
  SAM3 mask (sam3_server.py) -> backproject masked depth -> PCA initial
  guess -> Open3D ICP against the known box mesh -> pose in the CAMERA frame.

That per-frame camera-frame pose is then composed with a live TF lookup
(base_frame -> camera_optical_frame) into the ARM's OWN base_link frame
("{prefix}base_link") -- NOT the mobile base's base_link -- because that's
the frame CuRobo's stock ur5e.yml actually plans in (see object_geometry.py
and floor_geometry.py's shared docstring reasoning; also matches
publish_floor_marker.py's existing convention). Composing through the arm's
own base only depends on the real robot's own joint-state-driven TF chain
(already validated by every joint-space CuRobo test) plus the hand-eye
calibration (already validated to ~3-4mm) -- it never touches the
mobile-base-to-arm mount transform.

Robustness without becoming a tracker: keeps retrying (fresh SAM3 call each
time) until --confirm-frames consecutive accepted estimates agree within
--pos-tolerance / --rot-tolerance-deg, then averages exactly that window
and stops. BoxIcpTracker's own prev_R sign-consistency (carried across
.estimate() calls on one shared instance) keeps the PCA flip-ambiguity from
producing spurious spread between these back-to-back attempts.

Writes:
  <out>/object_pose.json               -- consumed by object_geometry.py
  <out>/pose_estimate/sample_NN.png    -- per accepted-frame debug overlay
                                           (SAM3 mask | box wireframe+axes)
  <out>/pose_estimate/final_summary.png -- last accepted frame + text readout

Prereqs: sam3_server.py already running (sam3 conda env, port 8420); the
D415 calib camera node running; the hand-eye extrinsic TF publisher running
(publish_camera_extrinsic.sh); the arm's robot_state_publisher up (for the
{prefix}base_link -> camera TF chain).

Usage:
    python3 estimate_object_pose.py --prefix ur5e --query "carton box"
"""
import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone

import cv2
import numpy as np
import requests
import trimesh
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from box_icp_ros_bridge import BoxIcpTracker, _draw_box_overlay

COLOR_TOPIC = "/d415_calib/d415_calib/color/image_raw"
DEPTH_TOPIC = "/d415_calib/d415_calib/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/d415_calib/d415_calib/color/camera_info"
# NOT the realsense driver's own "d415_calib_color_optical_frame" -- see
# publish_camera_extrinsic.sh: that exact name is also published internally
# by the camera driver itself, and two publishers broadcasting conflicting
# parents for the same child frame makes tf2 nondeterministically resolve
# through whichever one arrived last (confirmed live 2026-09-16: lookups
# intermittently failed with "two unconnected trees"). This is the
# distinctly-named, geometrically-identical synonym the hand-eye extrinsic
# now publishes instead.
CAMERA_FRAME = "d415_calib_color_optical_frame_handeye"
MESH_FILE = "/home/ws/nbv_scratch/cereal_box/mesh/textured_simple.obj"

OUT_DIR = "/home/ws/nbv_scratch/cereal_box"
DEBUG_DIR = os.path.join(OUT_DIR, "pose_estimate")
POSE_PATH = os.path.join(OUT_DIR, "object_pose.json")


class Capture(Node):
    def __init__(self, base_frame):
        super().__init__("estimate_object_pose")
        self.bridge = CvBridge()
        self.color = None
        self.depth_m = None
        self.K = None
        self.base_frame = base_frame
        self.create_subscription(Image, COLOR_TOPIC, self._color_cb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self._depth_cb, 10)
        self.create_subscription(CameraInfo, INFO_TOPIC, self._info_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _color_cb(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _depth_cb(self, msg):
        depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        self.depth_m = depth_mm.astype(np.float32) / 1e3

    def _info_cb(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def lookup_base_to_cam(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.base_frame, CAMERA_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None


def wait_for(node, predicate, timeout_s, what):
    t0 = time.time()
    while not predicate() and time.time() - t0 < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not predicate():
        print(f"ERROR: timed out waiting for {what}")
        sys.exit(1)


def tf_to_matrix(tf_msg):
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def make_sam3_overlay(cv_img, mask, query, found, score):
    overlay = cv_img.copy()
    mbool = mask.astype(bool)
    overlay[mbool] = (0.5 * overlay[mbool] + 0.5 * np.array([60, 60, 255])).astype(np.uint8)
    cv2.putText(overlay, f'"{query}" found={found} score={score:.2f}', (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if found else (0, 0, 255), 2)
    return overlay


def call_sam3(server_url, color_bgr, query, threshold):
    ok, jpg = cv2.imencode(".jpg", color_bgr)
    if not ok:
        return False, 0.0, np.zeros(color_bgr.shape[:2], dtype=np.uint8)
    try:
        resp = requests.post(
            server_url, files={"image": ("frame.jpg", jpg.tobytes(), "image/jpeg")},
            data={"query": query, "threshold": threshold}, timeout=10.0,
        )
    except requests.exceptions.RequestException as e:
        print(f"  ! SAM3 request failed: {e}")
        return False, 0.0, np.zeros(color_bgr.shape[:2], dtype=np.uint8)
    if resp.status_code != 200:
        print(f"  ! SAM3 server returned HTTP {resp.status_code} (not a real 'not found' "
              f"result -- check --server-url and that the server is serving /segment): "
              f"{resp.text[:200]}")
        return False, 0.0, np.zeros(color_bgr.shape[:2], dtype=np.uint8)
    found = resp.headers.get("X-Sam3-Found", "0") == "1"
    score = float(resp.headers.get("X-Sam3-Score", "0.0"))
    mask = (
        cv2.imdecode(np.frombuffer(resp.content, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    )
    return found, score, mask


def pairwise_spread(translations, rotations):
    """Max pairwise position distance (m) and rotation angle (deg) across a
    list of (t, R) samples -- what actually gates acceptance, not just the
    latest sample's own confidence."""
    pos_spread = 0.0
    rot_spread = 0.0
    for i in range(len(translations)):
        for j in range(i + 1, len(translations)):
            pos_spread = max(pos_spread, float(np.linalg.norm(translations[i] - translations[j])))
            rel = rotations[i].T @ rotations[j]
            rot_spread = max(rot_spread, float(Rotation.from_matrix(rel).magnitude()) * 180.0 / np.pi)
    return pos_spread, rot_spread


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--prefix", default="ur5e", choices=["ur5", "ur5e"],
                     help="ur5e for the live robot (default), ur5 for the mock")
    ap.add_argument("--query", default="carton box",
                     help='SAM3 text prompt -- prefer shape/material terms over product '
                          'category (see project notes: "cereal box" fails on a plain back panel)')
    ap.add_argument("--server-url", default="http://localhost:8420")
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--mesh-file", default=MESH_FILE)
    ap.add_argument("--confirm-frames", type=int, default=3,
                     help="consecutive accepted estimates that must agree before accepting")
    ap.add_argument("--pos-tolerance", type=float, default=0.02,
                     help="max pairwise position spread (m) across the confirm window")
    ap.add_argument("--rot-tolerance-deg", type=float, default=10.0,
                     help="max pairwise rotation spread (deg) across the confirm window")
    ap.add_argument("--max-tries", type=int, default=30)
    ap.add_argument("--period-s", type=float, default=0.5,
                     help="minimum time between SAM3 calls (it's ~1-1.1s/call anyway)")
    args = ap.parse_args()
    args.server_url = args.server_url.rstrip("/") + "/segment"

    os.makedirs(DEBUG_DIR, exist_ok=True)
    mesh = trimesh.load(args.mesh_file)
    dims = tuple(mesh.extents.tolist())
    tracker = BoxIcpTracker(dims)
    base_frame = f"{args.prefix}base_link"

    rclpy.init()
    node = Capture(base_frame)

    wait_for(node, lambda: node.color is not None, 10, COLOR_TOPIC)
    wait_for(node, lambda: node.depth_m is not None, 10, DEPTH_TOPIC)
    wait_for(node, lambda: node.K is not None, 10, INFO_TOPIC)
    wait_for(node, lambda: node.lookup_base_to_cam() is not None, 10,
              f"TF {base_frame} -> {CAMERA_FRAME}")

    print(f'Estimating object pose in "{base_frame}" frame. query="{args.query}", '
          f'dims(sorted)={sorted(dims)}, need {args.confirm_frames} agreeing frames '
          f'(pos<{args.pos_tolerance * 1000:.0f}mm, rot<{args.rot_tolerance_deg:.0f}deg).')

    accepted = []  # list of dicts: t, R, overlay_img, score
    tries = 0
    last_sam3_call = 0.0
    while tries < args.max_tries:
        now = time.time()
        if now - last_sam3_call < args.period_s:
            rclpy.spin_once(node, timeout_sec=0.1)
            continue
        rclpy.spin_once(node, timeout_sec=0.1)
        last_sam3_call = time.time()
        tries += 1

        color = node.color.copy()
        depth_m = node.depth_m.copy()
        K = node.K.copy()
        tf = node.lookup_base_to_cam()
        if tf is None:
            print(f"  [{tries}] TF lookup failed, retrying")
            continue
        T_base_cam = tf_to_matrix(tf)

        found, score, mask = call_sam3(args.server_url, color, args.query, args.threshold)
        sam3_vis = make_sam3_overlay(color, mask, args.query, found, score)
        if not found:
            print(f"  [{tries}] SAM3 found nothing (score={score:.2f})")
            continue

        result = tracker.estimate(depth_m, mask, K)
        if result is None:
            print(f"  [{tries}] SAM3 found it but too few valid depth points, skipping")
            continue
        T_cam_obj, pts = result
        T_base_obj = T_base_cam @ T_cam_obj
        t_base = T_base_obj[:3, 3]
        R_base = T_base_obj[:3, :3]

        box_vis = _draw_box_overlay(color, T_cam_obj, tracker.dims, K)
        overlay_img = np.hstack([sam3_vis, box_vis])

        print(f"  [{tries}] accepted candidate: score={score:.2f}, "
              f"t_base={t_base.round(3).tolist()}, n_pts={len(pts)}")
        accepted.append({"t": t_base, "R": R_base, "overlay": overlay_img, "score": score})

        window = accepted[-args.confirm_frames:]
        if len(window) < args.confirm_frames:
            continue
        pos_spread, rot_spread = pairwise_spread([s["t"] for s in window], [s["R"] for s in window])
        print(f"       last {args.confirm_frames} agree? pos_spread={pos_spread * 1000:.1f}mm, "
              f"rot_spread={rot_spread:.1f}deg")
        if pos_spread <= args.pos_tolerance and rot_spread <= args.rot_tolerance_deg:
            print(f"  -> stable after {tries} tries ({len(window)} agreeing frames).")
            break

    node.destroy_node()
    rclpy.shutdown()

    if not accepted:
        print("ERROR: never got a single accepted SAM3+ICP estimate. Is the object "
              "actually visible? Try a different --query.")
        sys.exit(1)
    window = accepted[-args.confirm_frames:]
    if len(window) < args.confirm_frames:
        print(f"ERROR: only {len(window)}/{args.confirm_frames} accepted estimates "
              f"within {args.max_tries} tries -- not writing a pose (avoid feeding a "
              f"single noisy estimate to the planner). Debug overlays saved below.")
        for i, s in enumerate(accepted):
            cv2.imwrite(os.path.join(DEBUG_DIR, f"failed_sample_{i:02d}.png"), s["overlay"])
        sys.exit(1)
    pos_spread, rot_spread = pairwise_spread([s["t"] for s in window], [s["R"] for s in window])
    if pos_spread > args.pos_tolerance or rot_spread > args.rot_tolerance_deg:
        print(f"ERROR: exhausted {args.max_tries} tries without {args.confirm_frames} "
              f"consecutive agreeing estimates (last window: pos_spread="
              f"{pos_spread * 1000:.1f}mm, rot_spread={rot_spread:.1f}deg) -- not writing "
              f"a pose. Debug overlays saved below.")
        for i, s in enumerate(accepted):
            cv2.imwrite(os.path.join(DEBUG_DIR, f"failed_sample_{i:02d}.png"), s["overlay"])
        sys.exit(1)

    t_mean = np.mean([s["t"] for s in window], axis=0)
    R_mean = Rotation.from_matrix(np.stack([s["R"] for s in window])).mean().as_matrix()
    q_mean_xyzw = Rotation.from_matrix(R_mean).as_quat().tolist()

    os.makedirs(DEBUG_DIR, exist_ok=True)
    for i, s in enumerate(window):
        cv2.imwrite(os.path.join(DEBUG_DIR, f"sample_{i:02d}.png"), s["overlay"])
    final = window[-1]["overlay"].copy()
    cv2.putText(final, f"ACCEPTED  pos_spread={pos_spread * 1000:.1f}mm  "
                        f"rot_spread={rot_spread:.1f}deg  n={len(window)}",
                (10, final.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imwrite(os.path.join(DEBUG_DIR, "final_summary.png"), final)

    out = {
        "frame_id": base_frame,
        "t_arm_base": t_mean.tolist(),
        "q_arm_base_xyzw": q_mean_xyzw,
        "dims_m": tracker.dims.tolist(),  # SORTED [thin, mid, wide] -- must match
                                           # t_arm_base/q_arm_base_xyzw's own axis
                                           # convention, consumed as-is by
                                           # object_geometry.py's CuRobo cuboid
        "mesh_file": args.mesh_file,
        "query": args.query,
        "n_confirm_frames": len(window),
        "n_tries": tries,
        "position_spread_m": pos_spread,
        "rotation_spread_deg": rot_spread,
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }
    with open(POSE_PATH, "w") as f:
        json.dump(out, f, indent=2)

    print(f"\nAccepted object pose in '{base_frame}':")
    print(f"  t = {t_mean.round(4).tolist()}")
    print(f"  q_xyzw = {[round(v, 4) for v in q_mean_xyzw]}")
    print(f"  spread: pos={pos_spread * 1000:.1f}mm, rot={rot_spread:.1f}deg over "
          f"{len(window)} frames ({tries} tries total)")
    print(f"Wrote {POSE_PATH}")
    print(f"Debug overlays: {DEBUG_DIR}/sample_*.png, {DEBUG_DIR}/final_summary.png")
    print("Next: run publish_object_marker.py to see this as a live RViz marker "
          "next to the arm, and curobo_plan_real_spike.py will now pick this up "
          "as a collision obstacle automatically (unless --no-object).")


if __name__ == "__main__":
    main()
