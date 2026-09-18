#!/usr/bin/env python3
"""Saves the latest camera frame to a fixed file, overwritten repeatedly, so it
can be viewed by just keeping that file open in VS Code / Nautilus / any image
viewer that refreshes on file change - avoids this container's broken X11
window mapping for GUI apps like image_view/rqt_image_view.

Usage: python3 live_frame_saver.py [--topic /d415_calib/d415_calib/color/image_raw]
                                    [--out /home/ws/nbv_scratch/live_frame.jpg]
                                    [--hz 2]
"""
import argparse
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class LiveFrameSaver(Node):
    def __init__(self, topic, out_path, hz):
        super().__init__("live_frame_saver")
        self.bridge = CvBridge()
        self.out_path = out_path
        self.min_period = 1.0 / hz
        self.last_save = 0.0
        self.count = 0
        self.sub = self.create_subscription(Image, topic, self.cb, 10)
        self.get_logger().info(f"Subscribed to {topic}, saving to {out_path} at up to {hz} Hz")

    def cb(self, msg):
        now = time.time()
        if now - self.last_save < self.min_period:
            return
        self.last_save = now
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(self.out_path, img)
        self.count += 1
        if self.count % 10 == 1:
            self.get_logger().info(f"saved frame #{self.count}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/d415_calib/d415_calib/color/image_raw")
    ap.add_argument("--out", default="/home/ws/nbv_scratch/live_frame.jpg")
    ap.add_argument("--hz", type=float, default=2.0)
    args = ap.parse_args()

    rclpy.init()
    node = LiveFrameSaver(args.topic, args.out, args.hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
