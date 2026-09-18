#!/usr/bin/env python3
"""
Publish the one-shot object pose estimate (estimate_object_pose.py's output)
as TWO RViz Markers, same reasoning as publish_floor_marker.py: CuRobo's
WorldConfig only exists inside the rob_env planning process, it's never on
the ROS graph by itself, so you can't otherwise SEE what the planner is
actually routing around.

Two markers, same pose, different sizes -- so you can see both the raw
estimate and the safety margin actually protecting it:
  /object_pose_marker       -- yellow, the RAW estimated box (no padding) --
                                "here's what SAM3+ICP thinks the object is"
  /object_obstacle_marker   -- red/translucent, the PADDED collision cuboid
                                actually fed to CuRobo (object_geometry.py's
                                OBJECT_MARGIN_M bigger on every side) --
                                "here's what the planner actually avoids"

Geometry comes from object_geometry.py -- the SAME module
curobo_plan_real_spike.py uses, so this always matches what's actually being
planned around. Published in the ARM's own base_link frame
("{prefix}base_link"), matching floor_geometry.py's convention and what
CuRobo actually plans in.

Run (after estimate_object_pose.py has written object_pose.json):
    source /home/ws/steve_env.sh
    python3 publish_object_marker.py --prefix ur5e

Then in RViz (e.g. alongside steve_live_rviz.rviz): Add -> By display type ->
Marker, Topic = /object_pose_marker, and again for /object_obstacle_marker.
"""
import argparse

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

from object_geometry import DEFAULT_OBJECT_POSE_PATH, OBJECT_MARGIN_M, load_object_pose


def make_marker(frame_id, marker_id, ns, t, q_xyzw, dims, color, alpha):
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = ns
    m.id = marker_id
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = t
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q_xyzw
    m.scale.x, m.scale.y, m.scale.z = dims
    m.color.r, m.color.g, m.color.b = color
    m.color.a = alpha
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", required=True, choices=["ur5", "ur5e"],
                     help="ur5e for the live robot, ur5 for the mock")
    ap.add_argument("--pose-file", default=DEFAULT_OBJECT_POSE_PATH)
    ap.add_argument("--margin", type=float, default=OBJECT_MARGIN_M)
    args = ap.parse_args()

    data = load_object_pose(args.pose_file)
    frame_id = f"{args.prefix}base_link"
    t = data["t_arm_base"]
    q_xyzw = data["q_arm_base_xyzw"]
    raw_dims = data["dims_m"]
    padded_dims = [d + 2 * args.margin for d in raw_dims]

    rclpy.init()
    node = Node("publish_object_marker")
    pose_pub = node.create_publisher(Marker, "/object_pose_marker", 10)
    obstacle_pub = node.create_publisher(Marker, "/object_obstacle_marker", 10)

    print(f"publishing object markers: frame_id={frame_id}")
    print(f"  /object_pose_marker      (yellow)  raw estimate, dims={[round(d, 3) for d in raw_dims]}")
    print(f"  /object_obstacle_marker  (red)     padded +{args.margin * 1000:.0f}mm, "
          f"dims={[round(d, 3) for d in padded_dims]}")
    print(f"  t={[round(v, 3) for v in t]}  q_xyzw={[round(v, 3) for v in q_xyzw]}")
    print("In RViz: Add -> By display type -> Marker, for each topic above.")
    print("Ctrl+C to stop.")

    try:
        while rclpy.ok():
            stamp = node.get_clock().now().to_msg()
            pose_marker = make_marker(frame_id, 0, "object_estimate", t, q_xyzw, raw_dims,
                                       color=(0.9, 0.8, 0.1), alpha=0.6)
            pose_marker.header.stamp = stamp
            pose_pub.publish(pose_marker)

            obstacle_marker = make_marker(frame_id, 1, "curobo_world", t, q_xyzw, padded_dims,
                                           color=(0.9, 0.2, 0.1), alpha=0.3)
            obstacle_marker.header.stamp = stamp
            obstacle_pub.publish(obstacle_marker)

            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
