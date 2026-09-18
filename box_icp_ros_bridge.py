#!/usr/bin/env python3
"""
Live 6D pose for a plain, texture-free rigid box (no front/back
distinction needed -- see steve-real-robot-nbv-port-plan memory: user
explicitly doesn't care which face is "front", only that the pose stays
CONSISTENT frame to frame) via classical geometry instead of
FoundationPose's learned model:

  SAM3 mask (already-running sam3_server.py) -> backproject masked depth
  to a 3D point cloud -> PCA for a rough orientation+position guess ->
  Open3D ICP against the known box mesh to refine it.

Runs entirely in system Python 3.10 (rclpy) -- no separate model server
needed, unlike FoundationPose/SAM3. open3d/trimesh installed --user into
system Python for this (numpy pinned to <2 afterward: open3d pulled in
numpy 2.x which breaks the apt-installed scipy's ABI expectations).

ASSUMPTION: the box is lying flat with its known-thinnest dimension
(0.03m) facing the camera as the surface normal -- i.e. resting on one of
its two largest (0.12 x 0.09) faces, not standing on an edge. Breaks if
the box is ever stood on its side.

Prereq: sam3_server.py already running (sam3 conda env, port 8420).

Usage:
    python3 box_icp_ros_bridge.py --query "cereal box"

Publishes:
    /boxpose/overlay  sensor_msgs/Image (bgr8) -- box wireframe + axes vis
    /boxpose/pose     geometry_msgs/PoseStamped -- ob_in_cam (meters)
    /sam3/overlay     sensor_msgs/Image (bgr8) -- SAM3 mask tint + score readout,
                      same style as sam3_ros_bridge.py's overlay, reusing the SAME
                      mask this script already fetches per frame (no extra SAM3 calls)
"""
import argparse
import time

import cv2
import numpy as np
import open3d as o3d
import requests
import trimesh
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

COLOR_TOPIC = "/d415_calib/d415_calib/color/image_raw"
DEPTH_TOPIC = "/d415_calib/d415_calib/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/d415_calib/d415_calib/color/camera_info"
MESH_FILE = "/home/ws/nbv_scratch/cereal_box/mesh/textured_simple.obj"


def _rotmat_to_quat_xyzw(R: np.ndarray) -> tuple:
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def _sample_box_surface_points(dims, n_per_face=250):
    """Uniform points on all 6 faces of a box centered at origin, local frame."""
    dx, dy, dz = dims
    pts = []
    for axis, half in enumerate((dx / 2, dy / 2, dz / 2)):
        for sign in (-1, 1):
            u = np.random.uniform(-1, 1, n_per_face)
            v = np.random.uniform(-1, 1, n_per_face)
            face = np.zeros((n_per_face, 3))
            other_axes = [a for a in range(3) if a != axis]
            other_halves = [d / 2 for i, d in enumerate((dx, dy, dz)) if i != axis]
            face[:, other_axes[0]] = u * other_halves[0]
            face[:, other_axes[1]] = v * other_halves[1]
            face[:, axis] = sign * half
            pts.append(face)
    return np.concatenate(pts, axis=0)


class BoxIcpTracker:
    """Pure geometry -- no rclpy/ROS deps, testable/reusable standalone."""

    LABELS = ("thin", "mid", "wide")

    def __init__(self, dims):
        self.dims = np.array(sorted(dims))  # ascending: [thin, mid, wide]
        # Canonical local axes are FIXED as (x=thin, y=mid, z=wide) -- must use
        # self.dims (sorted), not the raw constructor arg, so this matches what
        # _pca_initial_guess assumes when it builds R's columns in that same
        # (thin, mid, wide) order.
        self.canonical_pts = _sample_box_surface_points(self.dims)
        self.canonical_pcd = o3d.geometry.PointCloud()
        self.canonical_pcd.points = o3d.utility.Vector3dVector(self.canonical_pts)
        self.prev_R = None

    def backproject(self, depth_m, mask, K):
        ys, xs = np.where(mask > 0)
        if len(xs) < 30:
            return None
        z = depth_m[ys, xs]
        valid = z > 0.05
        if valid.sum() < 30:
            return None
        xs, ys, z = xs[valid], ys[valid], z[valid]
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        x = (xs - cx) * z / fx
        y = (ys - cy) * z / fy
        return np.stack([x, y, z], axis=1)

    def _isolate_dominant_face(self, pts, distance_threshold=0.004):
        """RANSAC-fit the single largest flat plane in `pts`, return only its
        inlier points. A real camera view is often not perfectly square-on to
        one face -- a corner view bends the point cloud across two faces,
        which breaks PCA's single-face-normal assumption in
        _pca_initial_guess (confirmed live 2026-09-16: a mask that included a
        sliver of a second face produced a visibly wrong rotation even after
        the axis-labeling fix). Discarding the non-dominant face's points
        here generalizes the estimator to arbitrary camera angles instead of
        requiring a careful square-on capture every time. Falls back to the
        full point set if the plane fit is degenerate (too few points to
        begin with, or too few inliers found)."""
        if len(pts) < 30:
            return pts
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        _plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold, ransac_n=3, num_iterations=500)
        if len(inliers) < 30:
            return pts
        return pts[inliers]

    def _pca_initial_guess(self, pts):
        centroid = pts.mean(axis=0)
        centered = pts - centroid
        cov = centered.T @ centered / len(pts)
        eigvals, eigvecs = np.linalg.eigh(cov)  # ascending eigenvalues
        normal = eigvecs[:, 0]
        axis_small = eigvecs[:, 1]
        axis_large = eigvecs[:, 2]

        # Face the normal toward the camera (camera is at origin looking down +z).
        if np.dot(normal, centroid) > 0:
            normal = -normal

        # Which face of the box is actually visible? Do NOT assume it's always
        # the two largest dims (i.e. do NOT assume the box lies flat on a
        # large face, thin-side-down) -- this object stands upright on its
        # SMALL base instead, so that assumption is false every time, not an
        # occasional edge case (confirmed live 2026-09-16: it produced a
        # visibly wrong wireframe). Measure the two in-plane extents and match
        # their ratio against all 3 possible visible-face dimension pairs;
        # whichever pair fits best tells us which known dimension is the
        # hidden/normal-axis one THIS time.
        extent_large = float((centered @ axis_large).ptp())
        extent_small = float((centered @ axis_small).ptp())
        observed_ratio = extent_large / max(extent_small, 1e-6)

        thin, mid, wide = self.dims  # e.g. 0.03, 0.09, 0.12
        dim = {"thin": thin, "mid": mid, "wide": wide}
        candidates = [("wide", "mid"), ("wide", "thin"), ("mid", "thin")]
        large_label, small_label = min(
            candidates, key=lambda c: abs((dim[c[0]] / dim[c[1]]) - observed_ratio))
        hidden_label = next(l for l in self.LABELS if l not in (large_label, small_label))

        axis_of = {hidden_label: normal, large_label: axis_large, small_label: axis_small}
        R = np.column_stack([axis_of["thin"], axis_of["mid"], axis_of["wide"]])
        # Enforce right-handedness (PCA eigenvectors don't guarantee it).
        if np.linalg.det(R) < 0:
            small_idx = self.LABELS.index(small_label)
            R[:, small_idx] *= -1

        if self.prev_R is not None:
            for col in range(3):
                if np.dot(self.prev_R[:, col], R[:, col]) < 0:
                    R[:, col] *= -1
            if np.linalg.det(R) < 0:  # a single-column flip can break handedness; refix
                R[:, 1] *= -1
        self.prev_R = R.copy()

        t = centroid - normal * (dim[hidden_label] / 2)
        return R, t

    def estimate(self, depth_m, mask, K):
        pts = self.backproject(depth_m, mask, K)
        if pts is None:
            return None
        # PCA's initial guess needs ONE flat face (see _isolate_dominant_face);
        # ICP below still registers against the FULL point set -- ICP matches
        # against the true canonical box shape (all 6 faces), so a second
        # face's sliver is real, useful signal for refinement once a good
        # initial guess gets it into the right basin, not noise to hide.
        face_pts = self._isolate_dominant_face(pts)
        if len(face_pts) < 30:
            return None
        R0, t0 = self._pca_initial_guess(face_pts)
        init = np.eye(4)
        init[:3, :3] = R0
        init[:3, 3] = t0

        observed_pcd = o3d.geometry.PointCloud()
        observed_pcd.points = o3d.utility.Vector3dVector(pts)

        result = o3d.pipelines.registration.registration_icp(
            self.canonical_pcd,
            observed_pcd,
            max_correspondence_distance=0.02,
            init=init,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30),
        )
        return result.transformation, pts


def _draw_box_overlay(img_bgr, pose, dims, K):
    dx, dy, dz = dims
    corners_local = np.array(
        [[sx * dx / 2, sy * dy / 2, sz * dz / 2] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    )
    R, t = pose[:3, :3], pose[:3, 3]
    corners_cam = (R @ corners_local.T).T + t
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    z = np.clip(corners_cam[:, 2], 1e-4, None)
    u = (corners_cam[:, 0] * fx / z + cx).astype(int)
    v = (corners_cam[:, 1] * fy / z + cy).astype(int)
    pts2d = np.stack([u, v], axis=1)

    edges = [(0, 1), (0, 2), (0, 4), (3, 1), (3, 2), (3, 7), (5, 1), (5, 4), (5, 7), (6, 2), (6, 4), (6, 7)]
    vis = img_bgr.copy()
    for a, b in edges:
        cv2.line(vis, tuple(pts2d[a]), tuple(pts2d[b]), (0, 255, 0), 2)

    axis_len = 0.08
    origin_cam = t
    for i, color in enumerate([(0, 0, 255), (0, 255, 0), (255, 0, 0)]):  # x=red, y=green, z=blue
        tip_cam = origin_cam + R[:, i] * axis_len
        ou = int(origin_cam[0] * fx / origin_cam[2] + cx)
        ov = int(origin_cam[1] * fy / origin_cam[2] + cy)
        tu = int(tip_cam[0] * fx / tip_cam[2] + cx)
        tv = int(tip_cam[1] * fy / tip_cam[2] + cy)
        cv2.line(vis, (ou, ov), (tu, tv), color, 3)
    return vis


class BoxIcpRosBridge(Node):
    def __init__(self, query, server_url, hz, dims, threshold):
        super().__init__("box_icp_ros_bridge")
        self.bridge = CvBridge()
        self.query = query
        self.server_url = server_url.rstrip("/") + "/segment"
        self.threshold = threshold
        self.min_period = 1.0 / hz
        self.last_run = 0.0
        self.tracker = BoxIcpTracker(dims)
        # SORTED (thin, mid, wide) -- must match the tracker's own axis
        # convention (see BoxIcpTracker.__init__), not the raw constructor
        # arg's arbitrary mesh.extents order, or _draw_box_overlay draws the
        # wireframe on the wrong axes.
        self.dims = self.tracker.dims
        self.K = None
        self.latest_depth = None
        self.frame_count = 0

        self.overlay_pub = self.create_publisher(Image, "/boxpose/overlay", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/boxpose/pose", 10)
        self.sam3_overlay_pub = self.create_publisher(Image, "/sam3/overlay", 10)
        self.split_pub = self.create_publisher(Image, "/boxpose/split_view", 10)

        self.create_subscription(CameraInfo, INFO_TOPIC, self._info_cb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self._depth_cb, 10)
        self.create_subscription(Image, COLOR_TOPIC, self.on_frame, 10)
        self.get_logger().info(f'query="{query}" up to {hz} Hz, dims={dims} -> {self.server_url}')

    def _info_cb(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _depth_cb(self, msg):
        depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        self.latest_depth = depth_mm.astype(np.float32) / 1e3

    def _publish_sam3_overlay(self, cv_img, mask, found, score, header):
        overlay = cv_img.copy()
        mbool = mask.astype(bool)
        overlay[mbool] = (0.5 * overlay[mbool] + 0.5 * np.array([60, 60, 255])).astype(np.uint8)
        cv2.putText(
            overlay,
            f'"{self.query}" found={found} score={score:.2f}',
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if found else (0, 0, 255),
            2,
        )
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        overlay_msg.header = header
        self.sam3_overlay_pub.publish(overlay_msg)
        return overlay

    def _publish_split(self, left, right, header):
        combined = np.hstack([left, right])
        combined_msg = self.bridge.cv2_to_imgmsg(combined, encoding="bgr8")
        combined_msg.header = header
        self.split_pub.publish(combined_msg)

    def on_frame(self, msg):
        now = time.time()
        if now - self.last_run < self.min_period:
            return
        if self.K is None or self.latest_depth is None:
            return
        self.last_run = now
        self.frame_count += 1

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok, jpg = cv2.imencode(".jpg", cv_img)
        if not ok:
            return

        try:
            resp = requests.post(
                self.server_url,
                files={"image": ("frame.jpg", jpg.tobytes(), "image/jpeg")},
                data={"query": self.query, "threshold": self.threshold},
                timeout=10.0,
            )
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"SAM3 server request failed: {e}")
            return
        found = resp.status_code == 200 and resp.headers.get("X-Sam3-Found", "0") == "1"
        score = float(resp.headers.get("X-Sam3-Score", "0.0")) if resp.status_code == 200 else 0.0
        mask = (
            cv2.imdecode(np.frombuffer(resp.content, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
            if resp.status_code == 200
            else np.zeros(cv_img.shape[:2], dtype=np.uint8)
        )
        sam3_vis = self._publish_sam3_overlay(cv_img, mask, found, score, msg.header)

        if not found:
            self.get_logger().warn("SAM3 found nothing this frame, skipping pose estimate")
            placeholder = cv_img.copy()
            cv2.putText(placeholder, "no pose (SAM3 found nothing)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self._publish_split(sam3_vis, placeholder, msg.header)
            return

        result = self.tracker.estimate(self.latest_depth, mask, self.K)
        if result is None:
            self.get_logger().warn("Not enough valid masked depth points, skipping")
            placeholder = cv_img.copy()
            cv2.putText(placeholder, "no pose (too few depth points)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self._publish_split(sam3_vis, placeholder, msg.header)
            return
        pose, _ = result

        vis = _draw_box_overlay(cv_img, pose, self.dims, self.K)
        overlay_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        overlay_msg.header = msg.header
        self.overlay_pub.publish(overlay_msg)
        self._publish_split(sam3_vis, vis, msg.header)

        R, t = pose[:3, :3], pose[:3, 3]
        qx, qy, qz, qw = _rotmat_to_quat_xyzw(R)
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = t.tolist()
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = (
            qx, qy, qz, qw,
        )
        self.pose_pub.publish(ps)

        if self.frame_count % 10 == 1:
            self.get_logger().info(f"frame #{self.frame_count}: pos={t.round(3).tolist()}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--query", default="cereal box")
    ap.add_argument("--server-url", default="http://localhost:8420")
    ap.add_argument("--hz", type=float, default=2.0)
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--mesh-file", default=MESH_FILE)
    args = ap.parse_args()

    mesh = trimesh.load(args.mesh_file)
    dims = tuple(mesh.extents.tolist())

    rclpy.init()
    node = BoxIcpRosBridge(args.query, args.server_url, args.hz, dims, args.threshold)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
