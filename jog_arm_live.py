#!/usr/bin/env python3
"""
Interactive per-joint jog for Steve's REAL UR5e arm (domain 74, onboard PC).

Same control path as jog_joint.py (single-point FollowJointTrajectory goals to
the active trajectory controller) but pointed at the live robot instead of the
mock: joint prefix "ur5e", controller /scaled_joint_trajectory_controller.
Every move you send here happens on the physical arm -- and shows up on
/tf immediately, so RViz (steve_live_rviz.rviz / launch_live_rviz.sh) mirrors
it live if you have it open.

Run:
    source /home/ws/steve_env.sh
    python3 jog_arm_live.py

Keys (identical to jog_joint.py):
    1..6      select joint (pan, lift, elbow, wrist_1, wrist_2, wrist_3)
    j / k     jog selected joint  - / +   by the current step
    [ / ]     halve / double the step size
    p         print current joint positions
    r         go to 'ready' pose      z   go to all-zeros
    s         stop (hold current position)
    q         quit

Safety:
    - Default step is small: 0.02 rad (~1.1 deg). [ / ] to adjust.
    - Default speed cap: 0.25 rad/s (slower than the mock script's 0.6) --
      override with --speed.
    - Before every send, checks /io_and_status_controller/robot_mode == RUNNING
      and .../safety_mode == NORMAL; refuses to send otherwise (won't silently
      queue a move against a protective/safeguard stop).
    - Requires typing "go" at startup before jogging is armed.
    - Nothing here bypasses the pendant E-stop. Keep it in reach.
    - If a send has no effect: the UR driver ALSO needs an "External Control"
      program node running on the pendant for real trajectory execution --
      dashboard/status alone (robot_mode RUNNING) is not sufficient.
"""
import sys
import termios
import tty
import math
import select
import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

PREFIX = "ur5e"
SHORT = ["pan", "lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
JOINTS = [f"{PREFIX}shoulder_pan_joint", f"{PREFIX}shoulder_lift_joint",
          f"{PREFIX}elbow_joint", f"{PREFIX}wrist_1_joint",
          f"{PREFIX}wrist_2_joint", f"{PREFIX}wrist_3_joint"]
CONTROLLER = "/scaled_joint_trajectory_controller"
LIMIT = 2 * math.pi
READY = [0.0, -2.094, 2.094, -1.571, -1.571, 0.0]

ROBOT_MODE_RUNNING = 7
SAFETY_MODE_NORMAL = 1


class Jog(Node):
    def __init__(self, speed):
        super().__init__("jog_arm_live")
        self.speed = speed
        self._js = None
        self._robot_mode = None
        self._safety_mode = None
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(
            RobotMode, "/io_and_status_controller/robot_mode", self._mode_cb, latched)
        self.create_subscription(
            SafetyMode, "/io_and_status_controller/safety_mode", self._safety_cb, latched)
        self._ac = ActionClient(
            self, FollowJointTrajectory, f"{CONTROLLER}/follow_joint_trajectory")

    def _js_cb(self, msg):
        self._js = msg

    def _mode_cb(self, msg):
        self._robot_mode = msg.mode

    def _safety_cb(self, msg):
        self._safety_mode = msg.mode

    def positions(self, timeout_s=5.0):
        end = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._js and all(j in self._js.name for j in JOINTS):
                idx = {n: i for i, n in enumerate(self._js.name)}
                return [self._js.position[idx[j]] for j in JOINTS]
        return None

    def safe_to_move(self):
        rclpy.spin_once(self, timeout_sec=0.2)
        if self._robot_mode != ROBOT_MODE_RUNNING:
            return False, f"robot_mode={self._robot_mode} (need {ROBOT_MODE_RUNNING}=RUNNING)"
        if self._safety_mode != SAFETY_MODE_NORMAL:
            return False, f"safety_mode={self._safety_mode} (need {SAFETY_MODE_NORMAL}=NORMAL)"
        return True, ""

    def send(self, target, duration):
        ok, why = self.safe_to_move()
        if not ok:
            print(f"\n  ! refusing to send: {why}")
            return
        if not self._ac.wait_for_server(timeout_sec=3.0):
            print("\n  ! action server not available -- is the bringup up?")
            return
        g = FollowJointTrajectory.Goal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINTS
        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in target]
        p.velocities = [0.0] * 6
        d = max(duration, 0.3)
        p.time_from_start = Duration(sec=int(d), nanosec=int((d % 1) * 1e9))
        g.trajectory.points = [p]
        f = self._ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=3.0)
        gh = f.result()
        if gh is None or not gh.accepted:
            print("\n  ! goal rejected")
            return
        # Block until this goal actually FINISHES executing (not just accepted)
        # before returning control to the caller. This is deliberate: sending a
        # new trajectory goal while the previous one is still running preempts
        # it mid-motion, and doing that rapidly (fast repeated key taps) can
        # overrun the UR driver's real-time RTDE read/write loop -- this is
        # exactly what caused the "Pipeline Producer overflowed" fault that
        # took controller_manager down. Blocking here serializes jogs: any key
        # pressed while a move is in progress just waits in the OS input
        # buffer and is handled after this one completes.
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=d + 2.0)
        if rf.result() is None:
            print("\n  ! timed out waiting for goal to finish (robot may still be moving)")

    def duration_for(self, cur, target):
        max_delta = max(abs(t - c) for t, c in zip(target, cur))
        return max(max_delta / self.speed, 0.3)


def getkey(timeout=0.1):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else None


def main():
    global CONTROLLER

    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.25,
                     help="rad/s cap for jog + ready/zero moves (default 0.25)")
    ap.add_argument("--controller", default=CONTROLLER)
    args = ap.parse_args()

    CONTROLLER = args.controller

    rclpy.init()
    node = Jog(speed=args.speed)
    print(f"using controller: {CONTROLLER}   speed cap: {args.speed} rad/s")

    cur = node.positions()
    if cur is None:
        print(f"no /joint_states with {PREFIX}* joints -- is the real robot bringup up?")
        return 1

    ok, why = node.safe_to_move()
    print(f"robot_mode={node._robot_mode}  safety_mode={node._safety_mode}"
          f"  {'OK' if ok else '-- ' + why}")
    print("current pose (rad):")
    for n, v in zip(JOINTS, cur):
        print(f"    {n:26s} {v:+.4f}")

    print("\nThis sends real trajectory goals to the physical arm.")
    print("Keep the pendant / E-stop within reach.")
    if input("Type 'go' to arm jogging: ").strip() != "go":
        print("aborted.")
        return 0

    sel = 1
    step = 0.02  # rad, ~1.1 deg

    def status():
        c = node.positions(timeout_s=1.0) or cur
        line = "  ".join(
            f"{'>' if i + 1 == sel else ' '}{i+1}:{SHORT[i]}={c[i]:+.3f}"
            for i in range(6))
        print(f"\r{line}   step={step:.3f}   ", end="", flush=True)

    print(__doc__)
    status()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            ch = getkey(0.1)
            if ch is None:
                continue
            if ch == "q":
                break
            elif ch in "123456":
                sel = int(ch)
            elif ch == "[":
                step = max(step / 2, 0.005)
            elif ch == "]":
                step = min(step * 2, 0.2)
            elif ch in ("j", "k"):
                c = node.positions(timeout_s=1.0)
                if c is None:
                    continue
                delta = step if ch == "k" else -step
                tgt = list(c)
                tgt[sel - 1] = max(-LIMIT, min(LIMIT, c[sel - 1] + delta))
                print()
                print(f"  jog {SHORT[sel-1]} {delta:+.3f} -> {tgt[sel-1]:+.3f}")
                node.send(tgt, abs(delta) / node.speed)
            elif ch == "p":
                c = node.positions(timeout_s=1.0) or cur
                print()
                for n, v in zip(JOINTS, c):
                    print(f"    {n:26s} {v:+.4f} rad  ({math.degrees(v):+7.2f} deg)")
            elif ch == "r":
                c = node.positions(timeout_s=1.0) or cur
                print("\n  -> ready pose")
                node.send(READY, node.duration_for(c, READY))
            elif ch == "z":
                c = node.positions(timeout_s=1.0) or cur
                print("\n  -> all zeros")
                node.send([0.0] * 6, node.duration_for(c, [0.0] * 6))
            elif ch == "s":
                c = node.positions(timeout_s=1.0)
                if c:
                    node.send(c, 0.3)
                print("\n  hold")
            status()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
