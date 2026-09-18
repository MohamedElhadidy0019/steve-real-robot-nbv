#!/usr/bin/env python3
"""
Passive watchdog on Steve's real UR5e arm pose -- read-only, sends nothing,
never touches the robot. Run it in its own terminal alongside anything else
(jog_arm_live.py, the pendant, RViz) to catch any joint motion you didn't
expect, from any source.

    source /home/ws/steve_env.sh
    python3 watch_arm_pose.py [--threshold-deg 0.3] [--heartbeat-s 10]

Watches /joint_states (the arm's own joint_state_broadcaster feed, ~400 Hz,
not the derived /tf) and prints:
  - a "moved" line the moment any joint's position changes by more than
    --threshold-deg since the last sample, naming which joint(s) and by how
    much, with a timestamp
  - a quiet "heartbeat" line every --heartbeat-s seconds so you know it's
    still alive and connected even when nothing is moving

It does not know or care WHO moved the arm -- your script, the pendant, or
gravity settling after a stop. That's the point: it's an independent witness,
not tied to whatever you think should be happening.
"""
import sys
import time
import math
import argparse
import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

PREFIX = "ur5e"
SHORT = ["pan", "lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
JOINTS = [f"{PREFIX}shoulder_pan_joint", f"{PREFIX}shoulder_lift_joint",
          f"{PREFIX}elbow_joint", f"{PREFIX}wrist_1_joint",
          f"{PREFIX}wrist_2_joint", f"{PREFIX}wrist_3_joint"]


def ts():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


class Watch(Node):
    def __init__(self, threshold_rad):
        super().__init__("watch_arm_pose")
        self.threshold = threshold_rad
        self.baseline = None      # last recorded pose, list[6]
        self.create_subscription(JointState, "/joint_states", self._cb, 50)

    def _cb(self, msg):
        if not all(j in msg.name for j in JOINTS):
            return  # partial message from another publisher (base/pan-tilt)
        idx = {n: i for i, n in enumerate(msg.name)}
        cur = [msg.position[idx[j]] for j in JOINTS]

        if self.baseline is None:
            self.baseline = cur
            print(f"[{ts()}] baseline: " +
                  "  ".join(f"{SHORT[i]}={cur[i]:+.4f}" for i in range(6)))
            return

        deltas = [c - b for c, b in zip(cur, self.baseline)]
        moved = [i for i, d in enumerate(deltas) if abs(d) > self.threshold]
        if moved:
            parts = "  ".join(
                f"{SHORT[i]}: {self.baseline[i]:+.4f} -> {cur[i]:+.4f} "
                f"({math.degrees(deltas[i]):+.2f} deg)" for i in moved)
            print(f"[{ts()}] MOVED  {parts}")
            self.baseline = cur


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--threshold-deg", type=float, default=0.3,
                     help="min change per joint to count as 'moved' (default 0.3 deg)")
    ap.add_argument("--heartbeat-s", type=float, default=10.0,
                     help="seconds between quiet status lines (default 10)")
    args = ap.parse_args()

    rclpy.init()
    node = Watch(threshold_rad=math.radians(args.threshold_deg))
    print(f"watching {PREFIX}* joints on /joint_states, "
          f"threshold={args.threshold_deg} deg, heartbeat={args.heartbeat_s}s")
    print("(read-only -- this script sends nothing to the robot)")

    last_beat = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.time()
            if now - last_beat >= args.heartbeat_s:
                last_beat = now
                if node.baseline is None:
                    print(f"[{ts()}] heartbeat: no data yet -- "
                          f"is the arm bringup up? (source /home/ws/steve_env.sh)")
                else:
                    b = node.baseline
                    print(f"[{ts()}] heartbeat: " +
                          "  ".join(f"{SHORT[i]}={b[i]:+.4f}" for i in range(6)))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
