#!/usr/bin/env python3
"""SAM3 text-prompted segmentation (Promptable Concept Segmentation) via the
`transformers`-integrated Sam3Model/Sam3Processor - NOT the facebookresearch/sam3
research repo, no repo clone needed. See steve-sam3-env-setup memory for why.

Run inside the `sam3` conda env:
    source ~/miniforge3/etc/profile.d/conda.sh && conda activate sam3
    python3 sam3_segmenter.py --image path.jpg --query bottle --out-overlay out.png
"""
import argparse
import os

import numpy as np
import torch
from PIL import Image


class Sam3Segmenter:
    def __init__(self, model_id="facebook/sam3", device=None):
        from transformers import Sam3Model, Sam3Processor

        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.model = Sam3Model.from_pretrained(model_id).to(self.device)
        self.model.eval()
        self.processor = Sam3Processor.from_pretrained(model_id)

    def segment(self, image, text_query, threshold=0.5, mask_threshold=0.5):
        """image: PIL.Image (RGB). text_query: short noun phrase, e.g. "bottle".

        Returns a list of {"mask": bool ndarray [H,W], "box": [x1,y1,x2,y2] in
        pixel coords, "score": float}, sorted best-first. Empty list if the
        query didn't match anything above `threshold`.
        """
        inputs = self.processor(images=image, text=text_query, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)
        results = self.processor.post_process_instance_segmentation(
            outputs,
            threshold=threshold,
            mask_threshold=mask_threshold,
            target_sizes=inputs.get("original_sizes").tolist(),
        )[0]

        items = []
        for mask, box, score in zip(results["masks"], results["boxes"], results["scores"]):
            mask_np = mask.detach().cpu().numpy().astype(bool)
            box_list = box.detach().cpu().numpy().tolist()
            items.append({"mask": mask_np, "box": box_list, "score": float(score)})
        items.sort(key=lambda d: d["score"], reverse=True)
        return items

    def segment_best(self, image, text_query, threshold=0.5, mask_threshold=0.5):
        """Convenience: single highest-scoring match, or None."""
        items = self.segment(image, text_query, threshold=threshold, mask_threshold=mask_threshold)
        return items[0] if items else None


def save_overlay(image, mask, out_path, color=(255, 60, 60), alpha=0.5):
    img_np = np.array(image.convert("RGB")).astype(np.float32)
    overlay = img_np.copy()
    overlay[mask] = (1 - alpha) * img_np[mask] + alpha * np.array(color, dtype=np.float32)
    Image.fromarray(overlay.astype(np.uint8)).save(out_path)


def main():
    ap = argparse.ArgumentParser(description="SAM3 text-prompted segmentation")
    ap.add_argument("--image", required=True, help="path to input image")
    ap.add_argument("--query", required=True, help='short text query, e.g. "bottle", "box"')
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--mask-threshold", type=float, default=0.5)
    ap.add_argument("--out-mask", default=None, help="save the best mask as a binary PNG")
    ap.add_argument("--out-overlay", default=None, help="save an overlay visualization")
    ap.add_argument("--view", action="store_true",
                     help="debug mode: IGNORE --threshold and save overlays for the top 3 "
                          "candidates regardless of score -- shows whether 'found nothing' "
                          "means zero candidates at all, or a best guess that just missed "
                          "the cutoff. Saved under /home/ws/nbv_scratch/sam3_debug/ (or "
                          "next to --out-overlay if given).")
    args = ap.parse_args()

    image = Image.open(args.image).convert("RGB")
    seg = Sam3Segmenter()

    if args.view:
        debug_items = seg.segment(image, args.query, threshold=0.0, mask_threshold=args.mask_threshold)
        top = debug_items[:3]
        if not top:
            print(f'--view: zero candidates at all for "{args.query}" (not even below threshold).')
            return
        out_dir = os.path.dirname(args.out_overlay) if args.out_overlay else "/home/ws/nbv_scratch/sam3_debug"
        os.makedirs(out_dir or ".", exist_ok=True)
        print(f'--view: top {len(top)} candidate(s) for "{args.query}" (real threshold={args.threshold}):')
        for i, it in enumerate(top):
            passes = "PASSES" if it["score"] >= args.threshold else "below"
            path = os.path.join(out_dir, f"view_top{i + 1}_score{it['score']:.2f}_{passes}.png")
            save_overlay(image, it["mask"], path)
            print(f"  [{i + 1}] score={it['score']:.3f} ({passes} threshold {args.threshold}) box={it['box']}")
            print(f"      -> {path}")
        return

    items = seg.segment(image, args.query, threshold=args.threshold, mask_threshold=args.mask_threshold)

    if not items:
        print(f'No "{args.query}" found above threshold {args.threshold}.')
        return

    print(f'Found {len(items)} match(es) for "{args.query}":')
    for i, it in enumerate(items):
        print(f"  [{i}] score={it['score']:.3f} box={it['box']}")

    best = items[0]
    if args.out_mask:
        Image.fromarray((best["mask"].astype(np.uint8) * 255)).save(args.out_mask)
        print(f"Saved best mask to {args.out_mask}")
    if args.out_overlay:
        save_overlay(image, best["mask"], args.out_overlay)
        print(f"Saved overlay to {args.out_overlay}")


if __name__ == "__main__":
    main()
