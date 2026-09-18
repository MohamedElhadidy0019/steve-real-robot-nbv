#!/usr/bin/env python3
"""
One-shot capture of a single rgb+depth+K frame for FoundationPose's
initial register() call. System Python 3.10 (rclpy) -- mirrors
handeye_capture.py's topic/subscription pattern.

Requires the D415 camera launched WITH align_depth.enable:=true so the
depth topic shares pixel grid + K with the color topic (FoundationPose's
render+crop pipeline assumes rgb/depth/K are all consistent).

Usage:
  python3 foundationpose_capture_bootstrap.py --out /home/ws/nbv_scratch/cereal_box/test_scene0
"""
import argparse
import os
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

COLOR_TOPIC = "/d415_calib/d415_calib/color/image_raw"
DEPTH_TOPIC = "/d415_calib/d415_calib/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/d415_calib/d415_calib/color/camera_info"


class BootstrapCapture(Node):
    def __init__(self):
        super().__init__("foundationpose_capture_bootstrap")
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.K = None
        self.create_subscription(Image, COLOR_TOPIC, self._color_cb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self._depth_cb, 10)
        self.create_subscription(CameraInfo, INFO_TOPIC, self._info_cb, 10)

    def _color_cb(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

    def _info_cb(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def ready(self):
        return self.color is not None and self.depth is not None and self.K is not None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="output test_scene_dir")
    ap.add_argument("--timeout-s", type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    node = BootstrapCapture()

    import time

    start = time.time()
    while not node.ready():
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.time() - start > args.timeout_s:
            print(
                f"ERROR: timed out waiting for {COLOR_TOPIC} / {DEPTH_TOPIC} / {INFO_TOPIC}. "
                "Is the camera launched with align_depth.enable:=true?",
                file=sys.stderr,
            )
            sys.exit(1)

    rgb_dir = os.path.join(args.out, "rgb")
    depth_dir = os.path.join(args.out, "depth")
    os.makedirs(rgb_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)

    cv2.imwrite(os.path.join(rgb_dir, "0.png"), node.color)
    cv2.imwrite(os.path.join(depth_dir, "0.png"), node.depth)
    np.savetxt(os.path.join(args.out, "cam_K.txt"), node.K, fmt="%.18e")

    print(f"Saved rgb/depth/cam_K.txt to {args.out}")
    print(f"color shape={node.color.shape} depth shape={node.depth.shape} dtype={node.depth.dtype}")
    print(f"depth min/max (raw units): {node.depth.min()} / {node.depth.max()}")
    print(f"K:\n{node.K}")
    print(f"\nNext: run sam3_segmenter.py on {rgb_dir}/0.png to produce masks/0.png")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
