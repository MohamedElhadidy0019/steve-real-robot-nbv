#!/usr/bin/env python3
"""
One-shot: read the arm's CURRENT joint state off /joint_states (live domain 74,
system Python 3.10 -- this is the only process that can speak rclpy) and write
it to a JSON file in CuRobo's expected format (unprefixed joint names, in the
canonical [pan, lift, elbow, wrist_1, wrist_2, wrist_3] order) so the CuRobo
planner (rob_env, Python 3.12 -- can't import rclpy at all, see project memory)
can pick it up as a start state without needing rclpy itself.

Read-only. Sends nothing to the robot.

Run:
    source /home/ws/steve_env.sh
    python3 ros_joint_state_dump.py --prefix ur5e --out /home/ws/nbv_scratch/current_joint_state.json
"""
import argparse
import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

CANONICAL = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


class Dump(Node):
    def __init__(self, prefix):
        super().__init__("joint_state_dump")
        self.prefix = prefix
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg):
        self.js = msg

    def wait(self, timeout_s=10.0):
        wanted = [f"{self.prefix}{j}" for j in CANONICAL]
        end = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.js and all(j in self.js.name for j in wanted):
                idx = {n: i for i, n in enumerate(self.js.name)}
                return [self.js.position[idx[j]] for j in wanted]
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", default="ur5e")
    ap.add_argument("--out", default="/home/ws/nbv_scratch/current_joint_state.json")
    args = ap.parse_args()

    rclpy.init()
    node = Dump(args.prefix)
    positions = node.wait()
    node.destroy_node()
    rclpy.shutdown()

    if positions is None:
        print(f"! no /joint_states with '{args.prefix}*' joints within timeout "
              f"-- is the bringup up?")
        return 1

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump({"joint_names": CANONICAL, "positions": positions,
                    "prefix": args.prefix}, f, indent=2)

    print(f"wrote {args.out}")
    for n, v in zip(CANONICAL, positions):
        print(f"    {n:22s} {v:+.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
