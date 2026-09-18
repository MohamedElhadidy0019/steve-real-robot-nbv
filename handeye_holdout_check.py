#!/usr/bin/env python3
"""Held-out validation for the solved hand-eye calibration.

Run this at NEW robot poses (not used in handeye_capture.py's dataset), with
the (possibly-moved) marker visible. Each run reconstructs the marker's pose
in base_link using the solved X from handeye_result.json + this pose's live
FK, and appends it to a running holdout log. Since the marker is physically
fixed for the duration of this check, all reconstructions should agree with
each other -- that agreement (or lack of it) IS the real validation, since
none of these poses were used to fit X.

Usage:
    python3 handeye_holdout_check.py [--note "..."] [--reset]
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
from scipy.spatial.transform import Rotation as R

IMAGE_TOPIC = "/d415_calib/d415_calib/color/image_raw"
CAMERA_INFO_TOPIC = "/d415_calib/d415_calib/color/camera_info"
BASE_FRAME = "base_link"
FLANGE_FRAME = "ur5etool0"

ARUCO_DICT = cv2.aruco.DICT_5X5_50
MARKER_ID = 0
MARKER_SIZE_M = 0.17

RESULT_FILE = "/home/ws/nbv_scratch/handeye_result.json"
HOLDOUT_FILE = "/home/ws/nbv_scratch/handeye_holdout_log.json"
CAPTURE_DIR = "/home/ws/nbv_scratch/logs/handeye_holdout"


def to_h(Rm, t):
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = np.asarray(t).flatten()
    return T


class Capture(Node):
    def __init__(self):
        super().__init__("handeye_holdout_check")
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
    ap.add_argument("--note", default="")
    ap.add_argument("--reset", action="store_true", help="clear the holdout log (start a fresh marker-position test)")
    args = ap.parse_args()

    if args.reset and os.path.exists(HOLDOUT_FILE):
        os.remove(HOLDOUT_FILE)
        print("Holdout log cleared.")

    if not os.path.exists(RESULT_FILE):
        print(f"ERROR: no solved calibration at {RESULT_FILE}. Run handeye_solve.py first.")
        sys.exit(1)
    with open(RESULT_FILE) as f:
        result = json.load(f)
    R_gc = R.from_quat(result["quaternion_xyzw"]).as_matrix()
    t_gc = np.array(result["translation_m"])
    T_gripper_cam = to_h(R_gc, t_gc)

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
        print(f"ERROR: marker id {MARKER_ID} not detected in this frame.")
        cv2.imwrite(os.path.join(CAPTURE_DIR, "last_failed_frame.png"), frame)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    idx = ids.flatten().tolist().index(MARKER_ID)
    marker_corners = corners[idx]
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers([marker_corners], MARKER_SIZE_M, K, D)
    rvec, tvec = rvecs[0][0], tvecs[0][0]
    R_tc, _ = cv2.Rodrigues(rvec)
    T_cam_target = to_h(R_tc, tvec)

    t = tf.transform.translation
    q = tf.transform.rotation
    R_gb = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    t_gb = np.array([t.x, t.y, t.z])
    T_base_gripper = to_h(R_gb, t_gb)

    T_base_target = T_base_gripper @ T_gripper_cam @ T_cam_target
    pos = T_base_target[:3, 3]
    quat = R.from_matrix(T_base_target[:3, :3]).as_quat()

    dist_m = float(np.linalg.norm(tvec))
    print(f"Marker detected, distance {dist_m:.3f} m.")
    print(f"Reconstructed marker pose in {BASE_FRAME}: "
          f"pos=[{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}] m")

    log = []
    if os.path.exists(HOLDOUT_FILE):
        with open(HOLDOUT_FILE) as f:
            log = json.load(f)

    entry = {
        "index": len(log),
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "note": args.note,
        "distance_m": dist_m,
        "base_target_position": pos.tolist(),
        "base_target_quaternion_xyzw": quat.tolist(),
    }
    log.append(entry)

    annotated = frame.copy()
    cv2.aruco.drawDetectedMarkers(annotated, [marker_corners], np.array([[MARKER_ID]]))
    cv2.drawFrameAxes(annotated, K, D, rvec, tvec, MARKER_SIZE_M * 0.5)
    img_path = os.path.join(CAPTURE_DIR, f"holdout_{entry['index']:03d}.png")
    cv2.imwrite(img_path, annotated)

    with open(HOLDOUT_FILE, "w") as f:
        json.dump(log, f, indent=2)

    if len(log) > 1:
        positions = np.array([e["base_target_position"] for e in log])
        mean_pos = positions.mean(axis=0)
        spread_mm = np.linalg.norm(positions - mean_pos, axis=1) * 1000
        print(f"\n{len(log)} held-out captures so far. Consistency check:")
        print(f"  mean marker position (m): [{mean_pos[0]:+.4f}, {mean_pos[1]:+.4f}, {mean_pos[2]:+.4f}]")
        print(f"  spread from mean (mm): min={spread_mm.min():.2f} max={spread_mm.max():.2f} mean={spread_mm.mean():.2f}")
        print(f"  (this spread is your real-world calibration error estimate -- lower is better)")
    else:
        print("\nFirst held-out capture logged. Move to a different pose and run again to get a consistency check.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
