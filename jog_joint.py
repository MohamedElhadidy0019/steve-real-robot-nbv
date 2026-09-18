#!/usr/bin/env python3
"""
Interactive per-joint jog for the UR5 on the MPO-700 (mock OR real).

Drives one joint at a time through joint_trajectory_controller -- the SAME control
path used on the real robot -- so the behaviour you observe in sim (RViz) is what
you get on hardware. Meant for the "move each joint alone and see how it behaves"
familiarisation step.

Run the bringup first:
    bash launch_mock.sh            # sim / mock
    # or the real-hardware bringup
then:
    source /home/ws/install/setup.bash
    python3 jog_joint.py

Keys:
    1..6      select joint (pan, lift, elbow, wrist_1, wrist_2, wrist_3)
    j / k     jog selected joint  - / +   by the current step
    [ / ]     halve / double the step size
    p         print current joint positions
    r         go to 'ready' pose      z   go to all-zeros
    s         stop (hold current position)
    q         quit

Each jog sends a short trajectory (step / speed seconds). Nothing is commanded
past +/- 2 pi. No collision checking in mock -- keep steps small near the cabinet.
"""
import sys
import termios
import tty
import math
import select

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

PREFIX = "ur5"
SHORT = ["pan", "lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
JOINTS = [f"{PREFIX}shoulder_pan_joint", f"{PREFIX}shoulder_lift_joint",
          f"{PREFIX}elbow_joint", f"{PREFIX}wrist_1_joint",
          f"{PREFIX}wrist_2_joint", f"{PREFIX}wrist_3_joint"]
# mock bringup -> joint_trajectory_controller is active.
# real UR bringup -> scaled_joint_trajectory_controller is active (respects the
# teach-pendant speed slider). Override with:  python3 jog_joint.py --controller /scaled_joint_trajectory_controller
CONTROLLER = "/joint_trajectory_controller"
LIMIT = 2 * math.pi
READY = [0.0, -2.094, 2.094, -1.571, -1.571, 0.0]
SPEED = 0.6  # rad/s -> trajectory duration = delta / SPEED


class Jog(Node):
    def __init__(self):
        super().__init__("jog_joint")
        self._js = None
        self.create_subscription(JointState, "/joint_states", self._cb, 10)
        self._ac = ActionClient(
            self, FollowJointTrajectory, f"{CONTROLLER}/follow_joint_trajectory")

    def _cb(self, msg):
        self._js = msg

    def positions(self, timeout_s=5.0):
        end = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._js and all(j in self._js.name for j in JOINTS):
                idx = {n: i for i, n in enumerate(self._js.name)}
                return [self._js.position[idx[j]] for j in JOINTS]
        return None

    def send(self, target, duration):
        if not self._ac.wait_for_server(timeout_sec=3.0):
            print("  ! action server not available -- is the bringup up?")
            return
        g = FollowJointTrajectory.Goal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINTS
        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in target]
        p.velocities = [0.0] * 6
        d = max(duration, 0.2)
        p.time_from_start = Duration(sec=int(d), nanosec=int((d % 1) * 1e9))
        g.trajectory.points = [p]
        f = self._ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=3.0)


def getkey(timeout=0.1):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else None


def main():
    global CONTROLLER
    if "--controller" in sys.argv:
        CONTROLLER = sys.argv[sys.argv.index("--controller") + 1]
    print(f"using controller: {CONTROLLER}")

    rclpy.init()
    node = Jog()
    cur = node.positions()
    if cur is None:
        print("no /joint_states with ur5 joints -- start the bringup first")
        return 1

    sel = 1          # 1-indexed joint
    step = 0.02      # rad  (~1.1 deg; use ] to grow, [ to shrink)

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
                step = min(step * 2, 1.0)
            elif ch in ("j", "k"):
                c = node.positions(timeout_s=1.0)
                if c is None:
                    continue
                delta = step if ch == "k" else -step
                tgt = list(c)
                tgt[sel - 1] = max(-LIMIT, min(LIMIT, c[sel - 1] + delta))
                print()
                print(f"  jog {SHORT[sel-1]} {delta:+.3f} -> {tgt[sel-1]:+.3f}")
                node.send(tgt, abs(delta) / SPEED)
            elif ch == "p":
                c = node.positions(timeout_s=1.0) or cur
                print()
                for n, v in zip(JOINTS, c):
                    print(f"    {n:26s} {v:+.4f} rad  ({math.degrees(v):+7.2f} deg)")
            elif ch == "r":
                print("\n  -> ready pose")
                node.send(READY, 4.0)
            elif ch == "z":
                print("\n  -> all zeros")
                node.send([0.0] * 6, 5.0)
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
