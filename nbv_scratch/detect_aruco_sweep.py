#!/usr/bin/env python3
"""One-shot: grab a frame from the D415 color topic and sweep all standard
ArUco dictionaries to identify which one (and which marker id) matches."""
import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

TOPIC = "/d415_calib/d415_calib/color/image_raw"

DICTS = {
    name: getattr(cv2.aruco, name)
    for name in dir(cv2.aruco)
    if name.startswith("DICT_")
}


class Grabber(Node):
    def __init__(self):
        super().__init__("aruco_sweep_grabber")
        self.bridge = CvBridge()
        self.frame = None
        self.sub = self.create_subscription(Image, TOPIC, self.cb, 10)

    def cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def main():
    rclpy.init()
    node = Grabber()
    import time
    t0 = time.time()
    while node.frame is None and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.frame is None:
        print("ERROR: no frame received on", TOPIC)
        sys.exit(1)
    frame = node.frame
    cv2.imwrite("/home/ws/nbv_scratch/logs/last_frame.png", frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    print(f"Frame grabbed: {frame.shape}. Sweeping {len(DICTS)} dictionaries...\n")
    hits = []
    for name, dict_id in sorted(DICTS.items()):
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        except Exception:
            continue
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
        if ids is not None and len(ids) > 0:
            hits.append((name, ids.flatten().tolist(), len(corners)))
            print(f"  MATCH  {name:22s} ids={ids.flatten().tolist()}")

    if not hits:
        print("\nNo dictionary matched. Saved frame to logs/last_frame.png for inspection.")
    else:
        print(f"\n{len(hits)} dictionary(ies) matched. Saved annotated frame too.")
        best_dict, best_ids, _ = hits[0]
        aruco_dict = cv2.aruco.getPredefinedDictionary(DICTS[best_dict])
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
        annotated = frame.copy()
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
        cv2.imwrite("/home/ws/nbv_scratch/logs/aruco_annotated.png", annotated)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
