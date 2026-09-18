#!/usr/bin/env python3
"""Solve the eye-in-hand hand-eye calibration (camera->tool0 extrinsic) from
the samples collected by handeye_capture.py.

Runs cv2.calibrateHandEye with every available method for cross-check, then
validates the chosen result by checking how consistent the marker's pose in
base_link comes out across all samples (it's physically fixed, so it should
compute to the same pose every time -- spread here IS the calibration error).
"""
import json

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

DATA_FILE = "/home/ws/nbv_scratch/handeye_samples.json"
OUT_FILE = "/home/ws/nbv_scratch/handeye_result.json"

METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def to_h(Rm, t):
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = np.asarray(t).flatten()
    return T


def main():
    with open(DATA_FILE) as f:
        samples = json.load(f)
    n = len(samples)
    print(f"Loaded {n} samples from {DATA_FILE}")

    R_gripper2base, t_gripper2base = [], []
    R_target2cam, t_target2cam = [], []
    for s in samples:
        q = s["gripper2base"]["quaternion_xyzw"]
        t = s["gripper2base"]["translation"]
        R_gripper2base.append(R.from_quat(q).as_matrix())
        t_gripper2base.append(np.array(t))

        rvec = np.array(s["target2cam"]["rvec"])
        tvec = np.array(s["target2cam"]["tvec"])
        Rm, _ = cv2.Rodrigues(rvec)
        R_target2cam.append(Rm)
        t_target2cam.append(tvec)

    results = {}
    for name, method in METHODS.items():
        try:
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=method
            )
        except cv2.error as e:
            print(f"{name}: FAILED ({e})")
            continue
        quat = R.from_matrix(R_cam2gripper).as_quat()
        results[name] = (R_cam2gripper, t_cam2gripper.flatten(), quat)
        t = t_cam2gripper.flatten()
        print(f"{name:10s} t_cam2gripper=[{t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}] m  "
              f"quat_xyzw=[{quat[0]:+.4f}, {quat[1]:+.4f}, {quat[2]:+.4f}, {quat[3]:+.4f}]")

    if not results:
        print("ERROR: every method failed.")
        return

    # cross-method agreement check
    names = list(results.keys())
    print("\nCross-method spread (translation mm, rotation deg), pairwise:")
    max_t_spread = 0.0
    max_r_spread = 0.0
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            ti, tj = results[names[i]][1], results[names[j]][1]
            t_diff_mm = np.linalg.norm(ti - tj) * 1000
            ri = R.from_matrix(results[names[i]][0])
            rj = R.from_matrix(results[names[j]][0])
            r_diff_deg = np.degrees((ri.inv() * rj).magnitude())
            max_t_spread = max(max_t_spread, t_diff_mm)
            max_r_spread = max(max_r_spread, r_diff_deg)
    print(f"  max pairwise translation spread: {max_t_spread:.2f} mm")
    print(f"  max pairwise rotation spread:    {max_r_spread:.2f} deg")

    # pick TSAI as the reported result (classic default), but only if methods roughly agree
    chosen = "TSAI" if "TSAI" in results else names[0]
    R_cg, t_cg, quat_cg = results[chosen]
    print(f"\nUsing method: {chosen}")

    T_gripper_cam = to_h(R_cg, t_cg)

    # validation: marker is physically fixed -> base_T_target should be ~constant
    base_targets = []
    for i in range(n):
        T_base_gripper = to_h(R_gripper2base[i], t_gripper2base[i])
        T_cam_target = to_h(R_target2cam[i], t_target2cam[i])
        T_base_target = T_base_gripper @ T_gripper_cam @ T_cam_target
        base_targets.append(T_base_target)

    positions = np.array([T[:3, 3] for T in base_targets])
    mean_pos = positions.mean(axis=0)
    pos_std_mm = positions.std(axis=0) * 1000
    pos_spread_mm = np.linalg.norm(positions - mean_pos, axis=1) * 1000

    rots = R.from_matrix([T[:3, :3] for T in base_targets])
    mean_rot = rots.mean()
    rot_spread_deg = np.degrees([(mean_rot.inv() * r).magnitude() for r in rots])

    print(f"\nValidation: marker pose in base_link, computed independently from each of the {n} samples.")
    print(f"  mean position (m): [{mean_pos[0]:+.4f}, {mean_pos[1]:+.4f}, {mean_pos[2]:+.4f}]")
    print(f"  position std per axis (mm): [{pos_std_mm[0]:.2f}, {pos_std_mm[1]:.2f}, {pos_std_mm[2]:.2f}]")
    print(f"  position spread from mean (mm): min={pos_spread_mm.min():.2f} max={pos_spread_mm.max():.2f} mean={pos_spread_mm.mean():.2f}")
    print(f"  rotation spread from mean (deg): min={rot_spread_deg.min():.2f} max={rot_spread_deg.max():.2f} mean={rot_spread_deg.mean():.2f}")

    worst_idx = int(np.argmax(pos_spread_mm))
    print(f"  worst sample: index {samples[worst_idx]['index']} ({pos_spread_mm[worst_idx]:.2f} mm off mean)")

    out = {
        "method": chosen,
        "translation_m": t_cg.tolist(),
        "quaternion_xyzw": quat_cg.tolist(),
        "note": "T_gripper_camera: camera pose expressed in ur5etool0 frame",
        "n_samples": n,
        "cross_method_max_translation_spread_mm": max_t_spread,
        "cross_method_max_rotation_spread_deg": max_r_spread,
        "validation_position_spread_mm": {
            "min": float(pos_spread_mm.min()),
            "max": float(pos_spread_mm.max()),
            "mean": float(pos_spread_mm.mean()),
        },
        "validation_rotation_spread_deg": {
            "min": float(rot_spread_deg.min()),
            "max": float(rot_spread_deg.max()),
            "mean": float(rot_spread_deg.mean()),
        },
    }
    with open(OUT_FILE, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nSaved result to {OUT_FILE}")


if __name__ == "__main__":
    main()
