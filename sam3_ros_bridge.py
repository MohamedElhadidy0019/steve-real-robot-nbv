#!/usr/bin/env python3
"""Orchestrator bridge: subscribes to a camera Image topic, sends each frame
(throttled) to the SAM3 server (sam3_server.py, running in the `sam3` conda
env - see steve-multi-env-pipeline-architecture memory), and publishes the
returned mask back onto the ROS graph. This is the piece that actually talks
rclpy, so it runs under SYSTEM Python 3.10 (rclpy is ABI-locked to it) - NOT
inside the sam3 conda env.

Prereq: sam3_server.py already running (python3 sam3_server.py, in the sam3
conda env). This script does not start it.

Usage:
    python3 sam3_ros_bridge.py --query bottle
    python3 sam3_ros_bridge.py --query box --topic /d415_calib/d415_calib/color/image_raw --hz 1.0

Publishes:
    /sam3/mask   sensor_msgs/Image (mono8, 0/255) - same size as the input frame
    /sam3/score  std_msgs/Float32 - confidence of the best match (0.0 if none found)
    /sam3/found  std_msgs/Bool - whether the query matched anything above threshold
"""
import argparse
import time

import cv2
import numpy as np
import requests
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class Sam3RosBridge(Node):
    def __init__(self, topic, query, server_url, hz, threshold, mask_threshold):
        super().__init__("sam3_ros_bridge")
        self.bridge = CvBridge()
        self.query = query
        self.server_url = server_url.rstrip("/") + "/segment"
        self.min_period = 1.0 / hz
        self.threshold = threshold
        self.mask_threshold = mask_threshold
        self.last_run = 0.0
        self.frame_count = 0
        self.request_count = 0

        self.mask_pub = self.create_publisher(Image, "/sam3/mask", 10)
        self.overlay_pub = self.create_publisher(Image, "/sam3/overlay", 10)
        self.score_pub = self.create_publisher(Float32, "/sam3/score", 10)
        self.found_pub = self.create_publisher(Bool, "/sam3/found", 10)

        self.sub = self.create_subscription(Image, topic, self.on_frame, 10)
        self.get_logger().info(
            f'Subscribed to {topic}, query="{query}", up to {hz} Hz -> {self.server_url}'
        )

    def on_frame(self, msg):
        now = time.time()
        if now - self.last_run < self.min_period:
            return
        self.last_run = now
        self.frame_count += 1

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok, jpg = cv2.imencode(".jpg", cv_img)
        if not ok:
            self.get_logger().warn("JPEG encode failed, skipping frame")
            return

        try:
            resp = requests.post(
                self.server_url,
                files={"image": ("frame.jpg", jpg.tobytes(), "image/jpeg")},
                data={
                    "query": self.query,
                    "threshold": self.threshold,
                    "mask_threshold": self.mask_threshold,
                },
                timeout=10.0,
            )
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"SAM3 server request failed: {e} - is sam3_server.py running?")
            return

        if resp.status_code != 200:
            self.get_logger().error(f"SAM3 server returned {resp.status_code}: {resp.text[:200]}")
            return

        self.request_count += 1
        found = resp.headers.get("X-Sam3-Found", "0") == "1"
        score = float(resp.headers.get("X-Sam3-Score", "0.0"))

        mask_img = cv2.imdecode(np.frombuffer(resp.content, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        mask_msg = self.bridge.cv2_to_imgmsg(mask_img, encoding="mono8")
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        overlay = cv_img.copy()
        mbool = mask_img.astype(bool)
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
        overlay_msg.header = msg.header
        self.overlay_pub.publish(overlay_msg)

        self.score_pub.publish(Float32(data=score))
        self.found_pub.publish(Bool(data=found))

        if self.request_count % 5 == 1:
            self.get_logger().info(
                f'frame #{self.frame_count} req #{self.request_count}: '
                f'found={found} score={score:.3f} query="{self.query}"'
            )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/d415_calib/d415_calib/color/image_raw")
    ap.add_argument("--query", required=True, help='e.g. "bottle", "box"')
    ap.add_argument("--server-url", default="http://localhost:8420")
    ap.add_argument("--hz", type=float, default=1.0, help="max rate to call the SAM3 server at")
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--mask-threshold", type=float, default=0.5)
    args = ap.parse_args()

    rclpy.init()
    node = Sam3RosBridge(
        args.topic, args.query, args.server_url, args.hz, args.threshold, args.mask_threshold
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
