#!/usr/bin/env python3
"""
Generate a simple untextured box mesh as a CAD-model stand-in for a
consumer package with no real CAD available (Lion chocolate & caramel
cereal bar, 40g, measured ~12 x 9 x 3 cm).

Usage:
  python3 make_box_mesh.py [--x 0.12] [--y 0.09] [--z 0.03] [--out mesh/textured_simple.obj]

Origin is at the box's own centroid, matching FoundationPose's
convention for its stock demo meshes (e.g. mustard0's textured_simple.obj
is centered on the object, not at a corner) -- get this wrong and every
downstream pose is silently offset by half the box's own dimension.
"""
import argparse
import os

import trimesh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=0.12, help="width (m)")
    ap.add_argument("--y", type=float, default=0.09, help="height (m)")
    ap.add_argument("--z", type=float, default=0.03, help="depth/thickness (m)")
    ap.add_argument("--out", default="/home/ws/nbv_scratch/cereal_box/mesh/textured_simple.obj")
    args = ap.parse_args()

    mesh = trimesh.creation.box(extents=[args.x, args.y, args.z])
    # trimesh.creation.box is already centered at the origin.

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    mesh.export(args.out)
    print(f"Wrote box mesh {args.x}x{args.y}x{args.z} m -> {args.out}")
    print(f"Bounds: {mesh.bounds.tolist()}")
    print(f"Extents: {mesh.extents.tolist()}")


if __name__ == "__main__":
    main()
