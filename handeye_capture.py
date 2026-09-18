#!/usr/bin/env python3
"""Hand-eye calibration sample capture.

Run this ONCE PER ROBOT POSE (like save_pose.py's pattern): move the arm to a
new pose with the marker visible, then run this script. It grabs the current
D415 color frame + camera_info, detects the ArUco marker, solves its pose via
estimatePoseSingleMarkers (-> target2cam), looks up base_link -> ur5etool0 via
TF (-> gripper2base), and appends both to a growing JSON dataset. Once you
have ~15-20 samples spanning varied ORIENTATIONS (not just positions), feed
the dataset into cv2.calibrateHandEye (separate solve script, not this one).

Usage:
    python3 handeye_capture.py [--note "some description"] [--dry-run]
"""
import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

IMAGE_TOPIC = "/d415_calib/d415_calib/color/image_raw"
CAMERA_INFO_TOPIC = "/d415_calib/d415_calib/color/camera_info"
BASE_FRAME = "base_link"
FLANGE_FRAME = "ur5etool0"

ARUCO_DICT = cv2.aruco.DICT_5X5_50
MARKER_ID = 0
MARKER_SIZE_M = 0.17  # measured black-square side length

DATA_FILE = "/home/ws/nbv_scratch/handeye_samples.json"
CAPTURE_DIR = "/home/ws/nbv_scratch/logs/handeye_captures"

EDGE_MARGIN_PX = 30  # warn if any marker corner is this close to the image border


class Capture(Node):
    def __init__(self):
        super().__init__("handeye_capture")
        self.bridge = CvBridge()
        self.frame = None
        self.cam_info = None
        self.create_subscription(Image, IMAGE_TOPIC, self._image_cb, 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._info_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _image_cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _info_cb(self, msg):
        self.cam_info = msg


def wait_for(node, predicate, timeout_s, what):
    t0 = time.time()
    while not predicate() and time.time() - t0 < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not predicate():
        print(f"ERROR: timed out waiting for {what}")
        sys.exit(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--note", default="", help="free-text note about this pose")
    ap.add_argument("--dry-run", action="store_true", help="detect + preview only, don't save a sample")
    args = ap.parse_args()

    os.makedirs(CAPTURE_DIR, exist_ok=True)

    rclpy.init()
    node = Capture()

    wait_for(node, lambda: node.frame is not None, 10, IMAGE_TOPIC)
    wait_for(node, lambda: node.cam_info is not None, 10, CAMERA_INFO_TOPIC)

    tf_ok = {"got": False}

    def have_tf():
        try:
            tf_ok["tf"] = node.tf_buffer.lookup_transform(BASE_FRAME, FLANGE_FRAME, rclpy.time.Time())
            tf_ok["got"] = True
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        return tf_ok["got"]

    wait_for(node, have_tf, 10, f"TF {BASE_FRAME} -> {FLANGE_FRAME}")
    tf = tf_ok["tf"]

    frame = node.frame.copy()
    K = np.array(node.cam_info.k).reshape(3, 3)
    D = np.array(node.cam_info.d)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

    if ids is None or MARKER_ID not in ids.flatten().tolist():
        print(f"ERROR: marker id {MARKER_ID} not detected in this frame. Not saving.")
        fail_path = os.path.join(CAPTURE_DIR, "last_failed_frame.png")
        cv2.imwrite(fail_path, frame)
        print(f"Saved the failed frame for inspection: {fail_path}")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    idx = ids.flatten().tolist().index(MARKER_ID)
    marker_corners = corners[idx]

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        [marker_corners], MARKER_SIZE_M, K, D
    )
    rvec = rvecs[0][0]
    tvec = tvecs[0][0]

    h, w = frame.shape[:2]
    pts = marker_corners[0]
    near_edge = any(
        px < EDGE_MARGIN_PX or px > w - EDGE_MARGIN_PX or py < EDGE_MARGIN_PX or py > h - EDGE_MARGIN_PX
        for px, py in pts
    )
    if near_edge:
        print(f"WARNING: a marker corner is within {EDGE_MARGIN_PX}px of the frame edge "
              f"(possible partial crop) -- consider backing off before capturing.")

    dist_m = float(np.linalg.norm(tvec))
    print(f"Marker {MARKER_ID} detected. Camera-to-marker distance ~{dist_m:.3f} m.")

    annotated = frame.copy()
    cv2.aruco.drawDetectedMarkers(annotated, [marker_corners], np.array([[MARKER_ID]]))
    cv2.drawFrameAxes(annotated, K, D, rvec, tvec, MARKER_SIZE_M * 0.5)

    if args.dry_run:
        preview_path = os.path.join(CAPTURE_DIR, "dry_run_preview.png")
        cv2.imwrite(preview_path, annotated)
        print(f"--dry-run: not saved. Preview: {preview_path}")
        node.destroy_node()
        rclpy.shutdown()
        return

    os.makedirs(os.path.dirname(DATA_FILE), exist_ok=True)
    samples = []
    if os.path.exists(DATA_FILE):
        with open(DATA_FILE) as f:
            samples = json.load(f)

    sample_idx = len(samples)
    image_file = os.path.join(CAPTURE_DIR, f"sample_{sample_idx:03d}.png")
    cv2.imwrite(image_file, annotated)

    t = tf.transform.translation
    q = tf.transform.rotation
    sample = {
        "index": sample_idx,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "note": args.note,
        "gripper2base": {
            "translation": [t.x, t.y, t.z],
            "quaternion_xyzw": [q.x, q.y, q.z, q.w],
        },
        "target2cam": {
            "rvec": rvec.tolist(),
            "tvec": tvec.tolist(),
        },
        "marker_id": MARKER_ID,
        "marker_size_m": MARKER_SIZE_M,
        "distance_m": dist_m,
        "near_edge_warning": near_edge,
        "image_file": image_file,
    }
    samples.append(sample)
    with open(DATA_FILE, "w") as f:
        json.dump(samples, f, indent=2)

    print(f"Saved sample #{sample_idx}. Total samples: {len(samples)}. Image: {image_file}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
