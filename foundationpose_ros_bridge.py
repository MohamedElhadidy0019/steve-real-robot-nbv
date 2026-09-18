#!/usr/bin/env python3
"""
Orchestrator bridge for live FoundationPose tracking. System Python 3.10
(rclpy) -- mirrors sam3_ros_bridge.py's design (see
steve-multi-env-pipeline-architecture memory).

Prereq: foundationpose_server.py already running (in the `foundationpose`
conda env) on --server-url. This script does not start it.

On startup, registers ONCE using a pre-captured bootstrap frame (rgb/depth/
mask/K saved by foundationpose_capture_bootstrap.py + sam3_segmenter.py --
see steve-real-robot-nbv-port-plan memory for why SAM3 and FoundationPose
never need to be GPU-resident at the same time: the mask is only needed for
that one frame). After that it subscribes to the LIVE color + aligned-depth
topics and calls /track on every throttled frame -- no mask needed per
frame, tracking carries forward from the previous pose.

Usage:
    python3 foundationpose_ros_bridge.py --bootstrap-dir /home/ws/nbv_scratch/cereal_box/test_scene0

Publishes:
    /foundationpose/overlay  sensor_msgs/Image (bgr8) -- box+axis vis, same style as run_demo.py
    /foundationpose/pose     geometry_msgs/PoseStamped -- ob_in_cam (meters)
"""
import argparse
import os
import time

import cv2
import numpy as np
import requests
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

COLOR_TOPIC = "/d415_calib/d415_calib/color/image_raw"
DEPTH_TOPIC = "/d415_calib/d415_calib/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/d415_calib/d415_calib/color/camera_info"


def _rotmat_to_quat_xyzw(R: np.ndarray) -> tuple:
    """Closed-form rotation-matrix -> quaternion, no extra ROS/scipy dep needed."""
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


class FoundationPoseRosBridge(Node):
    def __init__(self, topic, depth_topic, server_url, hz, track_refine_iter):
        super().__init__("foundationpose_ros_bridge")
        self.bridge = CvBridge()
        self.server_url = server_url.rstrip("/")
        self.min_period = 1.0 / hz
        self.track_refine_iter = track_refine_iter
        self.last_run = 0.0
        self.frame_count = 0
        self.request_count = 0
        self.k_str = None
        self.latest_depth = None

        self.overlay_pub = self.create_publisher(Image, "/foundationpose/overlay", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/foundationpose/pose", 10)

        self.create_subscription(CameraInfo, INFO_TOPIC, self._info_cb, 10)
        self.create_subscription(Image, depth_topic, self._depth_cb, 10)
        self.color_sub = self.create_subscription(Image, topic, self.on_frame, 10)
        self.get_logger().info(f"Subscribed to {topic} + {depth_topic}, up to {hz} Hz -> {self.server_url}")

    def _info_cb(self, msg):
        self.k_str = ",".join(f"{v:.10f}" for v in msg.k)

    def _depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

    def register_from_bootstrap(self, bootstrap_dir: str, est_refine_iter: int) -> bool:
        rgb_path = os.path.join(bootstrap_dir, "rgb", "0.png")
        depth_path = os.path.join(bootstrap_dir, "depth", "0.png")
        mask_path = os.path.join(bootstrap_dir, "masks", "0.png")
        k_path = os.path.join(bootstrap_dir, "cam_K.txt")
        for p in (rgb_path, depth_path, mask_path, k_path):
            if not os.path.exists(p):
                self.get_logger().error(f"Missing bootstrap file: {p}")
                return False

        k_str = ",".join(f"{v:.10f}" for v in np.loadtxt(k_path).reshape(-1))

        with open(rgb_path, "rb") as fr, open(depth_path, "rb") as fd, open(mask_path, "rb") as fm:
            resp = requests.post(
                f"{self.server_url}/register",
                files={
                    "rgb": ("rgb.png", fr.read(), "image/png"),
                    "depth": ("depth.png", fd.read(), "image/png"),
                    "mask": ("mask.png", fm.read(), "image/png"),
                },
                data={"k": k_str, "est_refine_iter": est_refine_iter},
                timeout=60.0,
            )
        if resp.status_code != 200:
            self.get_logger().error(f"/register failed {resp.status_code}: {resp.text[:300]}")
            return False

        self._publish_response(resp, header_frame_id="d415_calib_color_optical_frame")
        self.get_logger().info("Registered from bootstrap frame. Now tracking live frames.")
        return True

    def on_frame(self, msg):
        now = time.time()
        if now - self.last_run < self.min_period:
            return
        if self.k_str is None or self.latest_depth is None:
            return
        self.last_run = now
        self.frame_count += 1

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok_rgb, rgb_png = cv2.imencode(".png", cv_img)
        ok_depth, depth_png = cv2.imencode(".png", self.latest_depth)
        if not (ok_rgb and ok_depth):
            self.get_logger().warn("PNG encode failed, skipping frame")
            return

        try:
            resp = requests.post(
                f"{self.server_url}/track",
                files={
                    "rgb": ("rgb.png", rgb_png.tobytes(), "image/png"),
                    "depth": ("depth.png", depth_png.tobytes(), "image/png"),
                },
                data={"k": self.k_str, "track_refine_iter": self.track_refine_iter},
                timeout=10.0,
            )
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"FoundationPose server request failed: {e}")
            return

        if resp.status_code != 200:
            self.get_logger().error(f"/track returned {resp.status_code}: {resp.text[:200]}")
            return

        self.request_count += 1
        self._publish_response(resp, header=msg.header)

        if self.request_count % 10 == 1:
            self.get_logger().info(f"frame #{self.frame_count} track #{self.request_count}")

    def _publish_response(self, resp, header=None, header_frame_id=None):
        overlay = cv2.imdecode(np.frombuffer(resp.content, dtype=np.uint8), cv2.IMREAD_COLOR)
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        if header is not None:
            overlay_msg.header = header
        elif header_frame_id is not None:
            overlay_msg.header.frame_id = header_frame_id
            overlay_msg.header.stamp = self.get_clock().now().to_msg()
        self.overlay_pub.publish(overlay_msg)

        pose_flat = np.array([float(v) for v in resp.headers["X-Fp-Pose"].split(",")])
        pose = pose_flat.reshape(4, 4)
        qx, qy, qz, qw = _rotmat_to_quat_xyzw(pose[:3, :3])

        ps = PoseStamped()
        ps.header = overlay_msg.header
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pose[:3, 3].tolist()
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = (
            qx, qy, qz, qw,
        )
        self.pose_pub.publish(ps)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bootstrap-dir", required=True, help="dir with rgb/0.png, depth/0.png, masks/0.png, cam_K.txt")
    ap.add_argument("--topic", default=COLOR_TOPIC)
    ap.add_argument("--depth-topic", default=DEPTH_TOPIC)
    ap.add_argument("--server-url", default="http://localhost:8421")
    ap.add_argument("--hz", type=float, default=5.0, help="max rate to call the FoundationPose server at")
    ap.add_argument("--est-refine-iter", type=int, default=5)
    ap.add_argument("--track-refine-iter", type=int, default=2)
    args = ap.parse_args()

    rclpy.init()
    node = FoundationPoseRosBridge(args.topic, args.depth_topic, args.server_url, args.hz, args.track_refine_iter)

    if not node.register_from_bootstrap(args.bootstrap_dir, args.est_refine_iter):
        node.get_logger().error("Bootstrap registration failed, exiting.")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
