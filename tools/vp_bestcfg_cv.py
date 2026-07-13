#!/usr/bin/env python3
"""Split-half cross-validated aggregation for the fair "best estimator per
detector" protocol (paper Section 6.2).

Input: one or more CSVs written by sweeplsd_vp_bestcfg (img,method,variant,err).
For each method independently:
  1. Sort image names; fold A = even indices, fold B = odd indices.
  2. For each fold as the TRAINING half, select the estimator variant with the
     lowest median error on that half.
  3. Evaluate the selected variant on the OTHER half; pool the two test halves
     (every image is scored exactly once, under a variant chosen without it).
  4. Report the pooled median and the pooled fraction below 2 deg / 5 deg.

This rule was verified to reproduce the 2026-07-06 study's published numbers
exactly from that study's surviving row CSVs (YUD 0.85/0.92/1.00, NYU
5.97/7.64/7.82, and the %<2 / %<5 columns).

Usage: vp_bestcfg_cv.py rows.csv [more_rows.csv ...] [--methods a,b,c]
"""
import csv
import statistics
import sys


def main(argv):
    paths, methods = [], None
    i = 1
    while i < len(argv):
        if argv[i] == "--methods":
            i += 1
            methods = argv[i].split(",")
        else:
            paths.append(argv[i])
        i += 1
    if not paths:
        print(__doc__)
        return 1

    data = {}  # method -> variant -> {img: err}
    for p in paths:
        with open(p) as f:
            for r in csv.DictReader(f):
                data.setdefault(r["method"], {}) \
                    .setdefault(r["variant"], {})[r["img"]] = float(r["err"])

    print(f"{'method':10s} {'medErr(CV)':>10s} {'<2deg':>7s} {'<5deg':>7s} "
          f"{'imgs':>5s}  picks (fold A train / fold B train)")
    for m in (methods or sorted(data)):
        dm = data[m]
        imgs = sorted(set.intersection(*[set(v) for v in dm.values()]))
        fa, fb = imgs[0::2], imgs[1::2]
        pooled, picks = [], []
        for train, test in ((fa, fb), (fb, fa)):
            best = min(sorted(dm),
                       key=lambda v: statistics.median([dm[v][im] for im in train]))
            picks.append(best)
            pooled += [dm[best][im] for im in test]
        med = statistics.median(pooled)
        lt2 = 100.0 * sum(1 for e in pooled if e < 2) / len(pooled)
        lt5 = 100.0 * sum(1 for e in pooled if e < 5) / len(pooled)
        print(f"{m:10s} {med:9.2f}° {lt2:6.1f}% {lt5:6.1f}% {len(imgs):5d}  "
              f"{picks[0]} / {picks[1]}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
