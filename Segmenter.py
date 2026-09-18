import cv2
import numpy as np
import torch
from segment_anything import SamAutomaticMaskGenerator, sam_model_registry

class Segmenter:
    def __init__(self, checkpoint_path = r"sam_vit_b_01ec64.pth", device=None):
        # initialize SAM model
        if device is None:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            self.device = device

        sam = sam_model_registry["vit_b"](checkpoint=checkpoint_path)
        sam.to(device=self.device)

        self.generator = SamAutomaticMaskGenerator(
            model=sam,
            points_per_side=32,
            pred_iou_thresh=0.86,
            stability_score_thresh=0.92,
            min_mask_region_area=500
        )

    def segment(self, image_rgb):
        #receive an RGB image and return a list of masks
        h, w, _ = image_rgb.shape
        masks_data = self.generator.generate(image_rgb)

        center_x, center_y = w / 2, h / 2
        best_main_mask = None
        min_dist = float('inf')

        for item in masks_data:
            mask = item['segmentation']
            area = item['area']

            #area thresholding
            if (h * w * 0.005) < area < (h * w * 0.7):
                moments = cv2.moments(mask.astype(np.uint8))
                if moments['m00'] != 0:
                    cx = int(moments['m10'] / moments['m00'])
                    cy = int(moments['m01'] / moments['m00'])
                    dist = np.sqrt((cx - center_x) ** 2 + (cy - center_y) ** 2)

                    if dist < min_dist:
                        min_dist = dist
                        best_main_mask = mask

        return best_main_mask

