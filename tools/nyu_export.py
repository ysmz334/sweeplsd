'''
Export NYU-VP (Kluger et al., CONSAC CVPR'20) into the same plain-text manifest
format that oplsd_yud (ref/20260612/bench/yud_eval) consumes, so the identical
Manhattan-frame rotation-error protocol can be run on an indoor dataset.

For each NYU image with a clean orthogonal vanishing-point triad we:
  * extract the RGB image from the NYU Depth v2 labelled MAT (v7.3 HDF5, via h5py),
  * read its GT vanishing points (data/vps_XXXX.csv), turn them into viewing
    directions (y-up camera frame to match yud_eval's (cy-y)/f calibration),
    pick the most-orthogonal triple and orthonormalise it (nearest rotation),
  * write a manifest row "name path R(row-major 3x3, columns = the 3 GT VP dirs)".
We also emit gtlines.txt (the hand-labelled GT line segments, data/
labelled_lines_XXXX.csv) so the estimator-ceiling row works as on YUD.

NYU RGB intrinsics (from nyu_vp/nyu.py).
Usage:
  python nyu_export.py --mat <nyu_labeled.mat> --vpdir <nyu_vp/data> \
      --imgdir <out images> --manifest <out manifest.txt> --gtlines <out gtlines.txt> \
      [--orth-th 0.30]
'''
import argparse, csv, glob, os, itertools
import numpy as np
import h5py
from PIL import Image

FX, FY = 5.1885790117450188e+02, 5.1946961112127485e+02
CX, CY = 3.2558244941119034e+02, 2.5373616633400465e+02


def vp_dir(X, Y):
    # y-up camera frame so it matches yud_eval's calibrate() ((cy - y)/f).
    d = np.array([(X - CX) / FX, (CY - Y) / FY, 1.0])
    return d / np.linalg.norm(d)


def read_vps(path):
    pts = []
    with open(path) as fh:
        r = csv.reader(fh, delimiter=' ')
        next(r, None)
        for row in r:
            row = [c for c in row if c != '']
            if len(row) >= 3:
                pts.append((float(row[1]), float(row[2])))
    return [vp_dir(X, Y) for X, Y in pts]


def vp_support(labelled_path, n_vps):
    # Number of labelled GT lines per vanishing-point index (= dominance of the VP).
    sup = [0] * n_vps
    if not os.path.exists(labelled_path):
        return sup
    with open(labelled_path) as fh:
        r = csv.reader(fh, delimiter=' ')
        next(r, None)
        for row in r:
            vals = [c for c in row if c != '']
            if not vals:
                continue
            vi = int(vals[0])
            nums = vals[1:]
            cnt = 0
            for k in range(0, len(nums) - 3, 4):
                try:
                    x1, y1, x2, y2 = map(float, nums[k:k + 4])
                except ValueError:
                    continue
                if (x1, y1) != (x2, y2):
                    cnt += 1
            if 0 <= vi < n_vps:
                sup[vi] += cnt
    return sup


def dominant_triad(dirs, support):
    # The GT camera frame is the room's dominant Manhattan triad: the 3 most-
    # supported VPs (= what a detector's many lines actually vote for). Returns
    # the triad and its orthogonality score (sum of |pairwise dot|, 0 = perfect).
    order = sorted(range(len(dirs)), key=lambda i: -support[i])[:3]
    a, b, d = (dirs[order[0]], dirs[order[1]], dirs[order[2]])
    s = abs(np.dot(a, b)) + abs(np.dot(a, d)) + abs(np.dot(b, d))
    return (a, b, d), s


def orthonormalise(triad):
    # Nearest rotation matrix (columns = the 3 dirs) via SVD.
    M = np.column_stack(triad)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R  # columns are orthonormal, close to the input dirs


def read_gtlines(path):
    segs = []
    with open(path) as fh:
        r = csv.reader(fh, delimiter=' ')
        next(r, None)
        for row in r:
            vals = [c for c in row if c != '']
            # row: idx then groups of 4 (x1 y1 x2 y2)
            nums = vals[1:]
            for k in range(0, len(nums) - 3, 4):
                try:
                    x1, y1, x2, y2 = (float(nums[k]), float(nums[k + 1]),
                                      float(nums[k + 2]), float(nums[k + 3]))
                except ValueError:
                    continue
                if (x1, y1) != (x2, y2):
                    segs.append((x1, y1, x2, y2))
    return segs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mat', required=True)
    ap.add_argument('--vpdir', required=True)
    ap.add_argument('--imgdir', required=True)
    ap.add_argument('--manifest', required=True)
    ap.add_argument('--gtlines', required=True)
    ap.add_argument('--orth-th', type=float, default=0.30,
                    help='max sum|pairwise dot| of the GT triple (0.30 ~= each pair within 5.7deg)')
    ap.add_argument('--skip-images', action='store_true',
                    help='reuse already-extracted PNGs (only rewrite manifest/gtlines)')
    args = ap.parse_args()
    os.makedirs(args.imgdir, exist_ok=True)

    vps_files = sorted(glob.glob(os.path.join(args.vpdir, 'vps_*.csv')))
    n = len(vps_files)
    print(f'NYU-VP: {n} images, orth-th={args.orth_th}')

    with h5py.File(args.mat, 'r') as mat:
        images = mat['images']  # HDF5 dims reversed from MATLAB 480x640x3xN
        print('images dataset shape:', images.shape, images.dtype)

        man = open(args.manifest, 'w')
        man.write(f'{0.5*(FX+FY):.6f} {CX:.6f} {CY:.6f}\n')
        gtl = open(args.gtlines, 'w')

        kept = 0
        for i in range(n):
            name = f'nyu{i:04d}'
            dirs = read_vps(vps_files[i])
            if len(dirs) < 3:
                continue
            lp = os.path.join(args.vpdir, f'labelled_lines_{i:04d}.csv')
            support = vp_support(lp, len(dirs))
            triad, score = dominant_triad(dirs, support)
            if score >= args.orth_th:
                continue
            R = orthonormalise(triad)  # columns = GT VP dirs (dominant Manhattan frame)

            # extract image i -> HxWx3 uint8
            ipath = os.path.abspath(os.path.join(args.imgdir, name + '.png'))
            if not args.skip_images:
                arr = images[i]                       # (3,640,480) typically
                img = np.transpose(arr, (2, 1, 0))    # -> (480,640,3)
                Image.fromarray(img.astype(np.uint8), 'RGB').save(ipath)

            # manifest row-major 3x3 (so yud_eval reads columns as the 3 GT dirs)
            m = ' '.join(f'{R[r, c]:.8f}' for r in range(3) for c in range(3))
            man.write(f'{name} {ipath} {m}\n')

            # ceiling GT lines
            lp = os.path.join(args.vpdir, f'labelled_lines_{i:04d}.csv')
            if os.path.exists(lp):
                segs = read_gtlines(lp)
                gtl.write(f'{name} {len(segs)}\n')
                for s in segs:
                    gtl.write(f'{s[0]:.2f} {s[1]:.2f} {s[2]:.2f} {s[3]:.2f}\n')
            kept += 1
            if kept % 200 == 0:
                print(f'  ...{kept} kept / {i+1} scanned')

        man.close(); gtl.close()
        print(f'kept {kept}/{n} images (clean orthogonal triad)')


if __name__ == '__main__':
    main()
