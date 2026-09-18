#!/usr/bin/env python3
"""Persistent SAM3 segmentation server. Loads the model ONCE at startup and
stays warm - the orchestrator (sam3_ros_bridge.py, system Python 3.10) POSTs
frames to it instead of importing SAM3 directly, since rclpy is ABI-locked to
Python 3.10 and SAM3 needs 3.12 (see steve-multi-env-pipeline-architecture
memory - this is that decided design, implemented).

Run inside the `sam3` conda env:
    source ~/miniforge3/etc/profile.d/conda.sh && conda activate sam3
    python3 sam3_server.py [--host 0.0.0.0] [--port 8420]

POST /segment
    multipart form fields:
      image: the image file (jpeg/png bytes)
      query: text prompt, e.g. "bottle"
      threshold: optional float, default 0.5
      mask_threshold: optional float, default 0.5
    Response: image/png - the single best-matching binary mask (0/255,
    same H x W as the input image). Headers carry the match metadata:
      X-Sam3-Score, X-Sam3-Box (comma-separated x1,y1,x2,y2), X-Sam3-Found (0/1)
    If nothing matched the query, returns an all-zero mask with X-Sam3-Found: 0.

GET /health -> {"status": "ok", "device": "cuda"|"cpu"}
"""
import argparse
import io

import numpy as np
import uvicorn
from fastapi import FastAPI, Form, UploadFile
from fastapi.responses import Response
from PIL import Image

from sam3_segmenter import Sam3Segmenter

app = FastAPI()
segmenter: Sam3Segmenter | None = None


@app.on_event("startup")
def _load_model():
    global segmenter
    print("Loading SAM3 model...")
    segmenter = Sam3Segmenter()
    print(f"SAM3 model loaded on {segmenter.device}, server ready.")


@app.get("/health")
def health():
    return {"status": "ok", "device": segmenter.device if segmenter else "not_loaded"}


@app.post("/segment")
async def segment(
    image: UploadFile,
    query: str = Form(...),
    threshold: float = Form(0.5),
    mask_threshold: float = Form(0.5),
):
    raw = await image.read()
    pil_image = Image.open(io.BytesIO(raw)).convert("RGB")
    w, h = pil_image.size

    best = segmenter.segment_best(pil_image, query, threshold=threshold, mask_threshold=mask_threshold)

    if best is None:
        mask_png = _mask_to_png(np.zeros((h, w), dtype=bool))
        return Response(
            content=mask_png,
            media_type="image/png",
            headers={"X-Sam3-Found": "0", "X-Sam3-Score": "0.0", "X-Sam3-Box": ""},
        )

    mask_png = _mask_to_png(best["mask"])
    box_str = ",".join(f"{v:.1f}" for v in best["box"])
    return Response(
        content=mask_png,
        media_type="image/png",
        headers={
            "X-Sam3-Found": "1",
            "X-Sam3-Score": f"{best['score']:.4f}",
            "X-Sam3-Box": box_str,
        },
    )


def _mask_to_png(mask_bool):
    buf = io.BytesIO()
    Image.fromarray((mask_bool.astype(np.uint8) * 255)).save(buf, format="PNG")
    return buf.getvalue()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8420)
    args = ap.parse_args()
    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
