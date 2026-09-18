#!/usr/bin/env python3
"""
Send a CuRobo-planned trajectory (JSON written by curobo_plan_real_spike.py) as
ONE multi-waypoint FollowJointTrajectory goal, to either the mock or the live
arm. Reuses jog_arm_live.py's proven pattern: block on goal ACCEPTANCE *and*
on the result (not just acceptance) before returning -- that wait-for-result
fix is what prevents the "Pipeline Producer overflowed" controller_manager
crash from the 2026-09-11 rapid-tap incident (see project memory). For --target
live, also reuses its robot_mode/safety_mode gate and its "type 'go'" arm step.

--target mock   PREFIX="ur5"   CONTROLLER=/joint_trajectory_controller
                 (neo_mpo_700-2 mock bringup + launch_rviz.sh -- no physical
                 risk, but note: mock kinematics are UR5/CB3-shaped, not
                 ur5e-shaped, so treat this as a rough sanity check of the
                 MOTION SHAPE and the send path, not a precise preview.)
--target live   PREFIX="ur5e"  CONTROLLER=/scaled_joint_trajectory_controller
                 (the physical arm. Requires robot_mode=RUNNING,
                 safety_mode=NORMAL, and typing 'go'. Keep the pendant/E-stop
                 in reach.)

Run:
    source /home/ws/steve_env.sh
    python3 ros_send_trajectory.py --traj /home/ws/nbv_scratch/planned_trajectory.json --target mock
    python3 ros_send_trajectory.py --traj /home/ws/nbv_scratch/planned_trajectory.json --target live
"""
import argparse
import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

TARGETS = {
    "mock": {"prefix": "ur5", "controller": "/joint_trajectory_controller", "safety": False},
    "live": {"prefix": "ur5e", "controller": "/scaled_joint_trajectory_controller", "safety": True},
}

ROBOT_MODE_RUNNING = 7
SAFETY_MODE_NORMAL = 1


class Sender(Node):
    def __init__(self, controller, use_safety):
        super().__init__("send_trajectory")
        self.controller = controller
        self.use_safety = use_safety
        self._robot_mode = None
        self._safety_mode = None
        if use_safety:
            from ur_dashboard_msgs.msg import RobotMode, SafetyMode
            latched = QoSProfile(depth=1)
            latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
            latched.reliability = ReliabilityPolicy.RELIABLE
            self.create_subscription(
                RobotMode, "/io_and_status_controller/robot_mode", self._mode_cb, latched)
            self.create_subscription(
                SafetyMode, "/io_and_status_controller/safety_mode", self._safety_cb, latched)
        self._ac = ActionClient(
            self, FollowJointTrajectory, f"{controller}/follow_joint_trajectory")

    def _mode_cb(self, msg):
        self._robot_mode = msg.mode

    def _safety_cb(self, msg):
        self._safety_mode = msg.mode

    def safe_to_move(self, timeout_s=5.0):
        if not self.use_safety:
            return True, ""
        # rclpy.spin_once() processes at most ONE ready callback per call --
        # right after node construction, discovery+delivery of TWO separate
        # latched topics (robot_mode, safety_mode) isn't guaranteed to have
        # happened yet, and even once discovered, a single spin_once can
        # service one topic's callback while leaving the other still
        # None. Loop until both arrive (or a real timeout elapses) instead
        # of trusting one brief spin. jog_arm_live.py never hit this because
        # its startup calls positions() first, which loops spin_once for up
        # to 5s waiting on /joint_states -- incidentally giving these two
        # subscriptions plenty of chances too. This class has no equivalent
        # warm-up, so it needs its own explicit loop.
        end = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while (self._robot_mode is None or self._safety_mode is None) and \
                self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._robot_mode != ROBOT_MODE_RUNNING:
            return False, f"robot_mode={self._robot_mode} (need {ROBOT_MODE_RUNNING}=RUNNING)"
        if self._safety_mode != SAFETY_MODE_NORMAL:
            return False, f"safety_mode={self._safety_mode} (need {SAFETY_MODE_NORMAL}=NORMAL)"
        return True, ""

    def send_trajectory(self, joint_names, positions, times):
        ok, why = self.safe_to_move()
        if not ok:
            print(f"! refusing to send: {why}")
            return False
        if not self._ac.wait_for_server(timeout_sec=5.0):
            print("! action server not available -- is the bringup up?")
            return False

        g = FollowJointTrajectory.Goal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = joint_names
        pts = []
        for pos, t in zip(positions, times):
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in pos]
            p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(p)
        g.trajectory.points = pts

        total_duration = times[-1] if times else 0.0
        print(f"sending {len(pts)} waypoints to {self.controller}, "
              f"total duration {total_duration:.2f}s ...")

        f = self._ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        gh = f.result()
        if gh is None or not gh.accepted:
            print("! goal rejected")
            return False

        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=total_duration + 5.0)
        if rf.result() is None:
            print("! timed out waiting for goal to finish (robot may still be moving)")
            return False

        print("done.")
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", required=True)
    ap.add_argument("--target", required=True, choices=list(TARGETS))
    ap.add_argument("--slow-factor", type=float, default=8.0,
                     help="stretch every waypoint's time_from_start by this much "
                          "(default 8x). This only ever REDUCES peak velocity/"
                          "acceleration/jerk vs CuRobo's planned timing -- it's a "
                          "pure time-reparametrization of the same path, so it can't "
                          "make the motion less safe, only slower. CuRobo optimizes "
                          "for speed by default, which is fine in sim but too fast "
                          "to trust blind on real hardware the first few times.")
    args = ap.parse_args()

    cfg = TARGETS[args.target]
    with open(args.traj) as f:
        traj = json.load(f)

    traj["time_from_start"] = [t * args.slow_factor for t in traj["time_from_start"]]
    print(f"slow_factor={args.slow_factor}x -> stretched duration "
          f"{traj['time_from_start'][-1]:.2f}s (planned was "
          f"{traj['time_from_start'][-1] / args.slow_factor:.2f}s)")

    joint_names = [f"{cfg['prefix']}{j}" for j in traj["joint_names"]]

    rclpy.init()
    node = Sender(cfg["controller"], cfg["safety"])

    if cfg["safety"]:
        ok, why = node.safe_to_move()
        print(f"target=live  robot_mode={node._robot_mode}  safety_mode={node._safety_mode}"
              f"  {'OK' if ok else '-- ' + why}")
        print("This sends a real, already-planned trajectory to the physical arm.")
        print("Keep the pendant / E-stop within reach.")
        if input("Type 'go' to execute: ").strip() != "go":
            print("aborted.")
            node.destroy_node()
            rclpy.shutdown()
            return 0
    else:
        print(f"target=mock  controller={cfg['controller']}  (no physical risk)")

    ok = node.send_trajectory(joint_names, traj["positions"], traj["time_from_start"])

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
