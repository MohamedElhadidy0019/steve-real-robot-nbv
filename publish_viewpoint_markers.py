#!/usr/bin/env python3
"""
Publish sample_object_viewpoints.py's reachability cache as an RViz
MarkerArray: one small sphere per candidate camera position, colored by
what the two-pass IK check found:
  GREEN            -- reachable AND collision-free (table + object counted)
  YELLOW           -- kinematically reachable, but every IK solution found
                       clips the table or the object
  RED (translucent) -- not reachable at all, even ignoring collision
Same "publish what the offline computation produced" pattern as
publish_object_marker.py / publish_floor_marker.py -- pure visualization of
a precomputed cache, no arm motion, no robot power needed. Only needs the
ROS graph up (TF/base_link), not the arm actually running.

Run (after sample_object_viewpoints.py has written the cache):
    source /home/ws/steve_env.sh
    python3 publish_viewpoint_markers.py --prefix ur5e

Then in RViz (e.g. alongside steve_live_rviz.rviz): Add -> By display type ->
MarkerArray, Topic = /viewpoint_markers.
"""
import argparse
import json

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

DEFAULT_CACHE_PATH = "/home/ws/nbv_scratch/cereal_box/viewpoint_cache.json"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", required=True, choices=["ur5", "ur5e"],
                     help="ur5e for the live robot, ur5 for the mock")
    ap.add_argument("--cache-file", default=DEFAULT_CACHE_PATH)
    ap.add_argument("--sphere-radius", type=float, default=0.015)
    args = ap.parse_args()

    with open(args.cache_file) as f:
        data = json.load(f)

    frame_id = f"{args.prefix}base_link"
    t_cand = data["t_candidates_camera"]
    reachable_kinematic = data["reachable_kinematic"]
    reachable_collision = data["reachable_collision_aware"]
    n_green = sum(reachable_collision)
    n_yellow = sum(k and not c for k, c in zip(reachable_kinematic, reachable_collision))
    n_red = len(t_cand) - n_green - n_yellow
    print(f"loaded {len(t_cand)} candidates from {args.cache_file}")
    print(f"  green  (reachable + collision-free): {n_green}")
    print(f"  yellow (reachable, blocked by table/object): {n_yellow}")
    print(f"  red    (not reachable at all): {n_red}")

    rclpy.init()
    node = Node("publish_viewpoint_markers")
    pub = node.create_publisher(MarkerArray, "/viewpoint_markers", 10)

    print(f"publishing /viewpoint_markers, frame_id={frame_id}. Ctrl+C to stop.")

    try:
        while rclpy.ok():
            arr = MarkerArray()
            stamp = node.get_clock().now().to_msg()
            for i, t in enumerate(t_cand):
                m = Marker()
                m.header.frame_id = frame_id
                m.header.stamp = stamp
                m.ns = "viewpoints"
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = t
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = args.sphere_radius * 2
                if reachable_collision[i]:
                    m.color.r, m.color.g, m.color.b, m.color.a = (0.1, 0.9, 0.1, 0.85)
                elif reachable_kinematic[i]:
                    m.color.r, m.color.g, m.color.b, m.color.a = (0.9, 0.85, 0.1, 0.7)
                else:
                    m.color.r, m.color.g, m.color.b, m.color.a = (0.9, 0.1, 0.1, 0.35)
                arr.markers.append(m)
            pub.publish(arr)
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
