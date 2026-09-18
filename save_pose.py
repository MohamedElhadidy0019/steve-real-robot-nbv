#!/usr/bin/env python3
"""
Append the REAL robot's CURRENT joint state to a growing list of named
benchmark poses. Run this once per pose, any time the arm is sitting where
you want to capture it.

Read-only. Sends nothing to the robot.

Run:
    source /home/ws/steve_env.sh
    python3 save_pose.py                # auto-named pose_1, pose_2, ...
    python3 save_pose.py --name front_left
"""
import argparse
import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

CANONICAL = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
PREFIX = "ur5e"
DEFAULT_FILE = "/home/ws/nbv_scratch/benchmark_poses.json"


class Dump(Node):
    def __init__(self):
        super().__init__("save_pose")
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg):
        self.js = msg

    def wait(self, timeout_s=10.0):
        wanted = [f"{PREFIX}{j}" for j in CANONICAL]
        end = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.js and all(j in self.js.name for j in wanted):
                idx = {n: i for i, n in enumerate(self.js.name)}
                return [self.js.position[idx[j]] for j in wanted]
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default=None, help="pose name (default: auto pose_N)")
    ap.add_argument("--file", default=DEFAULT_FILE)
    args = ap.parse_args()

    rclpy.init()
    node = Dump()
    positions = node.wait()
    node.destroy_node()
    rclpy.shutdown()

    if positions is None:
        print("! no /joint_states with 'ur5e*' joints within timeout -- is the bringup up?")
        return 1

    poses = []
    if os.path.exists(args.file):
        with open(args.file) as f:
            poses = json.load(f)

    name = args.name or f"pose_{len(poses) + 1}"
    if any(p["name"] == name for p in poses):
        print(f"! a pose named '{name}' already exists -- pick a different --name")
        return 1

    poses.append({"name": name, "joint_names": CANONICAL, "positions": positions})

    os.makedirs(os.path.dirname(args.file), exist_ok=True)
    with open(args.file, "w") as f:
        json.dump(poses, f, indent=2)

    print(f"saved '{name}' ({len(poses)} pose(s) total in {args.file})")
    for n, v in zip(CANONICAL, positions):
        print(f"    {n:22s} {v:+.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
