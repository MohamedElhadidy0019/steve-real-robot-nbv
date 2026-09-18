#!/usr/bin/env python3
"""
Persistent FoundationPose server. Loads the model + a fixed mesh ONCE at
startup and stays warm. Mirrors sam3_server.py's design (see
steve-multi-env-pipeline-architecture memory): rclpy is ABI-locked to
system Python 3.10, FoundationPose needs its own conda env (Python 3.11,
torch cu124) -- so the ROS-facing orchestrator (foundationpose_ros_bridge.py)
POSTs frames here instead of importing FoundationPose directly.

Run inside the `foundationpose` conda env:
    source ~/miniforge3/etc/profile.d/conda.sh && conda activate foundationpose
    cd /home/ws/FoundationPose  # must run from here, imports are relative
    python3 /home/ws/foundationpose_server.py --mesh-file /home/ws/nbv_scratch/cereal_box/mesh/textured_simple.obj

Stateful by design: POST /register must be called once (first frame) before
any POST /track calls -- track_one() internally uses the estimator's own
last-known pose as its starting hypothesis, exactly like run_demo.py's
i==0 vs else branch.

POST /register   multipart: rgb (png), depth (16-bit png, mm), mask (png, 0/255),
                  k (form, 9 comma-separated row-major floats), est_refine_iter (form, default 5)
POST /track       multipart: rgb (png), depth (16-bit png, mm),
                  k (form, 9 comma-separated row-major floats), track_refine_iter (form, default 2)
  Both return: image/png (rgb + box + axis overlay, same visual style as run_demo.py's
  debug>=1 vis), header X-Fp-Pose = 16 comma-separated row-major floats (ob_in_cam, meters)

GET /health -> {"status": "ok"|"not_registered", "device": ...}
"""
import argparse
import io
import os
import sys

import numpy as np
import uvicorn
from fastapi import FastAPI, Form, HTTPException, UploadFile
from fastapi.responses import Response
from PIL import Image

FOUNDATIONPOSE_REPO = "/home/ws/FoundationPose"
os.chdir(FOUNDATIONPOSE_REPO)  # estimater.py's `from Utils import *` needs this dir on sys.path
sys.path.insert(0, FOUNDATIONPOSE_REPO)

from estimater import *  # noqa: E402  (FoundationPose, ScorePredictor, PoseRefinePredictor, draw_*, dr, trimesh, cv2, np, set_logging_format, set_seed)

app = FastAPI()

STATE = {"est": None, "to_origin": None, "bbox": None, "registered": False}


def _load_estimator(mesh_file: str, debug_dir: str):
    set_logging_format()
    set_seed(0)
    mesh = trimesh.load(mesh_file)
    to_origin, extents = trimesh.bounds.oriented_bounds(mesh)
    bbox = np.stack([-extents / 2, extents / 2], axis=0).reshape(2, 3)

    os.makedirs(debug_dir, exist_ok=True)
    scorer = ScorePredictor()
    refiner = PoseRefinePredictor()
    glctx = dr.RasterizeCudaContext()
    est = FoundationPose(
        model_pts=mesh.vertices,
        model_normals=mesh.vertex_normals,
        mesh=mesh,
        scorer=scorer,
        refiner=refiner,
        debug_dir=debug_dir,
        debug=1,
        glctx=glctx,
    )
    return est, to_origin, bbox


def _decode_rgb(raw_bytes) -> np.ndarray:
    bgr = cv2.imdecode(np.frombuffer(raw_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    # np.ascontiguousarray: torch.as_tensor (used deep inside FoundationPose) rejects the
    # negative-stride view that a bare [..., ::-1] channel-reverse produces.
    return np.ascontiguousarray(bgr[..., ::-1])  # -> RGB, matching imageio.imread's convention


def _decode_depth_m(raw_bytes) -> np.ndarray:
    depth_mm = cv2.imdecode(np.frombuffer(raw_bytes, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    depth = depth_mm.astype(np.float32) / 1e3
    depth[(depth < 0.001) | (depth >= np.inf)] = 0
    return depth


def _decode_mask(raw_bytes) -> np.ndarray:
    m = cv2.imdecode(np.frombuffer(raw_bytes, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    return m.astype(bool)


def _parse_k(k_str: str) -> np.ndarray:
    vals = [float(v) for v in k_str.split(",")]
    if len(vals) != 9:
        raise HTTPException(400, f"k must be 9 comma-separated values, got {len(vals)}")
    return np.array(vals, dtype=np.float64).reshape(3, 3)


def _render_vis(rgb_rgb: np.ndarray, K: np.ndarray, pose: np.ndarray) -> bytes:
    to_origin, bbox = STATE["to_origin"], STATE["bbox"]
    center_pose = pose @ np.linalg.inv(to_origin)
    vis = draw_posed_3d_box(K, img=rgb_rgb, ob_in_cam=center_pose, bbox=bbox)
    vis = draw_xyz_axis(
        vis, ob_in_cam=center_pose, scale=0.1, K=K, thickness=3, transparency=0, is_input_rgb=True
    )
    ok, png = cv2.imencode(".png", np.ascontiguousarray(vis[..., ::-1]))  # back to BGR for cv2 encode
    if not ok:
        raise HTTPException(500, "PNG encode failed")
    return png.tobytes()


def _pose_header(pose: np.ndarray) -> str:
    return ",".join(f"{v:.8f}" for v in pose.reshape(-1))


@app.on_event("startup")
def _startup():
    args = app.state.cli_args
    print(f"Loading FoundationPose (mesh={args.mesh_file})...")
    est, to_origin, bbox = _load_estimator(args.mesh_file, args.debug_dir)
    STATE["est"] = est
    STATE["to_origin"] = to_origin
    STATE["bbox"] = bbox
    print("FoundationPose loaded, server ready. Call /register first.")


@app.get("/health")
def health():
    return {
        "status": "ok" if STATE["est"] is not None else "not_loaded",
        "registered": STATE["registered"],
    }


@app.post("/register")
async def register(
    rgb: UploadFile,
    depth: UploadFile,
    mask: UploadFile,
    k: str = Form(...),
    est_refine_iter: int = Form(5),
):
    if STATE["est"] is None:
        raise HTTPException(503, "model not loaded yet")
    K = _parse_k(k)
    rgb_rgb = _decode_rgb(await rgb.read())
    depth_m = _decode_depth_m(await depth.read())
    ob_mask = _decode_mask(await mask.read())

    pose = STATE["est"].register(K=K, rgb=rgb_rgb, depth=depth_m, ob_mask=ob_mask, iteration=est_refine_iter)
    STATE["registered"] = True

    png = _render_vis(rgb_rgb, K, pose)
    return Response(content=png, media_type="image/png", headers={"X-Fp-Pose": _pose_header(pose)})


@app.post("/track")
async def track(
    rgb: UploadFile,
    depth: UploadFile,
    k: str = Form(...),
    track_refine_iter: int = Form(2),
):
    if STATE["est"] is None:
        raise HTTPException(503, "model not loaded yet")
    if not STATE["registered"]:
        raise HTTPException(400, "call /register first (no prior pose to track from)")
    K = _parse_k(k)
    rgb_rgb = _decode_rgb(await rgb.read())
    depth_m = _decode_depth_m(await depth.read())

    pose = STATE["est"].track_one(rgb=rgb_rgb, depth=depth_m, K=K, iteration=track_refine_iter)

    png = _render_vis(rgb_rgb, K, pose)
    return Response(content=png, media_type="image/png", headers={"X-Fp-Pose": _pose_header(pose)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mesh-file", required=True)
    ap.add_argument("--debug-dir", default="/home/ws/nbv_scratch/cereal_box/fp_debug")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8421)
    args = ap.parse_args()

    app.state.cli_args = args
    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
