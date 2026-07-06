#!/usr/bin/env python
"""Export York Urban Database ground truth to a plain-text manifest the C++
evaluator (oplsd_yud) can read, mirroring how edlines_real feeds via text.

We export ONLY the reliable ground truth: the shared camera intrinsics and, per
image, the orthogonal Manhattan vanishing-point frame (vp_orthogonal, a 3x3
orthonormal matrix whose columns are the three mutually-orthogonal scene-axis
directions expressed in camera coordinates). The hand-labelled line segments in
LinesAndVP.mat are deliberately NOT exported: they are an incomplete,
Manhattan-only annotation and are not a trustworthy line-detection ground truth.
The evaluation instead feeds each detector's own lines into the same calibrated
Manhattan-frame estimator and compares the estimated frame to this GT.

Usage:
    python tools/yud_export.py <YorkUrbanDB_dir> <out_manifest.txt>

Manifest format:
    f cx cy
    <name> <jpg_path> r00 r01 r02 r10 r11 r12 r20 r21 r22      (one per image)
where r.. is vp_orthogonal row-major; the three GT directions are its columns.

Also writes <manifest_dir>/gtlines.txt with the hand-labelled GT line segments
per image (consecutive-row endpoint pairs):
    <name> <nlines>
    x0 y0 x1 y1   (x nlines)
These are used ONLY to validate the estimator itself (feeding GT lines through
the same VP pipeline gives the instrument's accuracy ceiling) — never to score
the detectors.
"""
import os
import sys

import numpy as np
import scipy.io as sio


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 1
    root, out_path = sys.argv[1], sys.argv[2]

    cam = sio.loadmat(os.path.join(root, "cameraParameters.mat"))
    focal_mm = float(np.array(cam["focal"]).ravel()[0])
    pixel_mm = float(np.array(cam["pixelSize"]).ravel()[0])
    pp = np.array(cam["pp"]).ravel()
    f_px = focal_mm / pixel_mm
    cx, cy = float(pp[0]), float(pp[1])
    print(f"intrinsics: f={f_px:.3f}px  cx={cx:.3f}  cy={cy:.3f}")

    dirs = sorted(d for d in os.listdir(root)
                  if os.path.isdir(os.path.join(root, d)) and d.startswith("P"))
    rows = []
    gtline_blocks = []
    skipped = 0
    for name in dirs:
        base = os.path.join(root, name, name)
        jpg = base + ".jpg"
        gt_path = base + "GroundTruthVP_Orthogonal_CamParams.mat"
        if not (os.path.exists(jpg) and os.path.exists(gt_path)):
            skipped += 1
            continue
        M = np.array(sio.loadmat(gt_path)["vp_orthogonal"], dtype=float)
        if M.shape != (3, 3):
            skipped += 1
            continue
        # sanity: orthonormal columns
        err = np.max(np.abs(M.T @ M - np.eye(3)))
        if err > 1e-2:
            print(f"  warn: {name} vp_orthogonal not orthonormal (err={err:.3g})")
        flat = " ".join(f"{v:.8f}" for v in M.reshape(-1))
        jpg_abs = os.path.abspath(jpg).replace("\\", "/")
        rows.append(f"{name} {jpg_abs} {flat}")

        # Hand-labelled GT line segments (consecutive-row endpoint pairs).
        lv = sio.loadmat(base + "LinesAndVP.mat")
        L = np.array(lv["lines"], dtype=float)
        nL = L.shape[0] // 2
        seg_lines = [f"{name} {nL}"]
        for k in range(nL):
            p0, p1 = L[2 * k], L[2 * k + 1]
            seg_lines.append(f"{p0[0]:.3f} {p0[1]:.3f} {p1[0]:.3f} {p1[1]:.3f}")
        gtline_blocks.append("\n".join(seg_lines))

    with open(out_path, "w") as o:
        o.write(f"{f_px:.6f} {cx:.6f} {cy:.6f}\n")
        o.write("\n".join(rows) + "\n")

    gt_out = os.path.join(os.path.dirname(out_path) or ".", "gtlines.txt")
    with open(gt_out, "w") as o:
        o.write("\n".join(gtline_blocks) + "\n")
    print(f"wrote {len(rows)} images to {out_path} (+ GT lines to {gt_out}, {skipped} skipped)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
