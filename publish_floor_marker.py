#!/usr/bin/env python3
"""
Publish CuRobo's floor/table obstacle as an RViz Marker, so you can actually
SEE the box the planner is routing around -- CuRobo's WorldConfig only exists
inside the planning process (rob_env), it's never on the ROS graph by itself.

Geometry comes from floor_geometry.py -- the SAME module curobo_plan_real_spike.py
uses, so this always matches what's actually being planned around (no separate
copy of the numbers to drift out of sync).

Published in the ARM's own base_link frame (matching what CuRobo actually
uses), NOT the mobile base or world frame -- pick --prefix to match whichever
graph you're pointed at: "ur5" for the mock (domain 0), "ur5e" for the live
robot (domain 74).

Run:
    # mock:
    ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 python3 publish_floor_marker.py --prefix ur5
    # live:
    source /home/ws/steve_env.sh && python3 publish_floor_marker.py --prefix ur5e

Then in RViz: Add -> By display type -> Marker, Topic = /floor_obstacle_marker.
"""
import argparse

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

from floor_geometry import DEFAULT_FLOOR_OFFSET, DEFAULT_FLOOR_SIZE, FLOOR_THICKNESS, floor_center_z


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", required=True, choices=["ur5", "ur5e"],
                     help="ur5 for the mock (domain 0), ur5e for the live robot (domain 74)")
    ap.add_argument("--floor-offset", type=float, default=DEFAULT_FLOOR_OFFSET)
    ap.add_argument("--floor-size", type=float, default=DEFAULT_FLOOR_SIZE)
    args = ap.parse_args()

    frame_id = f"{args.prefix}base_link"
    center_z = floor_center_z(args.floor_offset, FLOOR_THICKNESS)

    rclpy.init()
    node = Node("publish_floor_marker")
    pub = node.create_publisher(Marker, "/floor_obstacle_marker", 10)

    m = Marker()
    m.header.frame_id = frame_id
    m.ns = "curobo_world"
    m.id = 0
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = 0.0
    m.pose.position.y = 0.0
    m.pose.position.z = center_z
    m.pose.orientation.w = 1.0
    m.scale.x = args.floor_size
    m.scale.y = args.floor_size
    m.scale.z = FLOOR_THICKNESS
    m.color.r = 0.9
    m.color.g = 0.2
    m.color.b = 0.1
    m.color.a = 0.35  # semi-transparent, so the robot model isn't hidden

    print(f"publishing floor marker: frame_id={frame_id}  "
          f"{args.floor_size}x{args.floor_size}m  top surface at "
          f"z={-args.floor_offset:.3f}m relative to {frame_id}  "
          f"(offset={args.floor_offset:.3f}m)")
    print("in RViz: Add -> By display type -> Marker, Topic = /floor_obstacle_marker")
    print("Ctrl+C to stop.")

    try:
        while rclpy.ok():
            m.header.stamp = node.get_clock().now().to_msg()
            pub.publish(m)
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
