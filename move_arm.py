#!/usr/bin/env python3
"""
Minimal UR5 arm mover for the MPO-700 mock/real bringup.

Sends a single-point FollowJointTrajectory goal to joint_trajectory_controller
(the controller that is active in mock mode). This is the real-robot equivalent
of the PyBullet sim's execute_joint_states() and is the primitive the NBV loop
will call to drive to each viewpoint.

Usage:
    # named poses
    python3 move_arm.py ready
    python3 move_arm.py zero
    python3 move_arm.py up

    # explicit 6 joint angles (rad), order: pan lift elbow w1 w2 w3
    python3 move_arm.py 0 -2.094 2.094 -1.571 -1.571 0 --time 5

    # print current joint positions and exit
    python3 move_arm.py --show
"""
import sys
import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

PREFIX = "ur5"
JOINTS = [f"{PREFIX}{j}" for j in (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)]
CONTROLLER = "/joint_trajectory_controller"

NAMED = {
    "zero":  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "up":    [0.0, -1.5708, 0.0, 0.0, 0.0, 0.0],
    # compact ready pose, matches ur_arm.urdf.xacro initial_positions
    "ready": [0.0, -2.094, 2.094, -1.571, -1.571, 0.0],
}


class ArmMover(Node):
    def __init__(self):
        super().__init__("move_arm")
        self._last_js = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self._client = ActionClient(
            self, FollowJointTrajectory, f"{CONTROLLER}/follow_joint_trajectory")

    def _js_cb(self, msg):
        self._last_js = msg

    def current_positions(self, timeout_s=5.0):
        end = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._last_js and all(j in self._last_js.name for j in JOINTS):
                idx = {n: i for i, n in enumerate(self._last_js.name)}
                return [self._last_js.position[idx[j]] for j in JOINTS]
        return None

    def move_to(self, target, move_time):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"action server {CONTROLLER}/follow_joint_trajectory not available")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in target]
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=int(move_time),
                                      nanosec=int((move_time % 1) * 1e9))
        goal.trajectory.points = [pt]

        self.get_logger().info(
            "moving to [" + ", ".join(f"{x:+.3f}" for x in target) +
            f"] over {move_time:.1f}s")
        send = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        gh = send.result()
        if not gh.accepted:
            self.get_logger().error("goal rejected")
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        code = res.result().result.error_code
        ok = code == 0
        self.get_logger().info("done" if ok else f"finished with error_code {code}")
        return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", nargs="*",
                    help="named pose (%s) or 6 joint angles in rad" % "/".join(NAMED))
    ap.add_argument("--time", type=float, default=4.0, help="move duration (s)")
    ap.add_argument("--show", action="store_true", help="print current joints and exit")
    args = ap.parse_args()

    rclpy.init()
    node = ArmMover()
    try:
        if args.show or not args.target:
            cur = node.current_positions()
            if cur is None:
                print("no /joint_states with ur5 joints (is the bringup running?)")
                return 1
            print("current joint positions (rad):")
            for n, p in zip(JOINTS, cur):
                print(f"  {n:26s} {p:+.4f}")
            if not args.target:
                return 0
            return 0

        if len(args.target) == 1 and args.target[0] in NAMED:
            target = NAMED[args.target[0]]
        elif len(args.target) == 6:
            target = [float(x) for x in args.target]
        else:
            print("give a named pose (%s) or exactly 6 angles" % "/".join(NAMED))
            return 2

        return 0 if node.move_to(target, args.time) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
