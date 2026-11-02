#!/usr/bin/env python3
# Copyright 2026, Andrew Gaspar
# SPDX-License-Identifier: BSL-1.0
"""
Summarise t_slam's timing.csv (SLAM_TIMING_STAT=1 SLAM_WRITE_CSVS=1, as monado-cli xreal-slam-replay
writes it to its out dir): Basalt's per-frame frontend (optical flow) and backend (VIO optimisation)
times against the camera frame interval, and the push-to-pose latency including any queueing.

The frontend and backend are separate threads, so each must stay under the frame interval on its
own for the tracker to keep up; a growing push-to-pose latency over the run means a queue is building.

usage: scripts/slam-timing-summary.py [--hz 30] <dir or timing.csv>...
"""

import argparse
import csv
import math
import os


def pct(v, p):
    v = sorted(v)
    k = (len(v) - 1) * p / 100.0
    lo = math.floor(k)
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def stage(rows, idx, a, b):
    return [(r[idx[b]] - r[idx[a]]) / 1e6 for r in rows]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hz", type=float, default=30.0, help="camera rate (default 30, XREAL Air 2 Ultra)")
    ap.add_argument("paths", nargs="+")
    args = ap.parse_args()
    budget = 1000.0 / args.hz

    print(f"frame interval {budget:.1f} ms; stage times in ms as p50 / p95 / p99 / max")
    print(f"{'run':30s} {'frames':>6s}  {'frontend':>27s}  {'backend':>27s}  {'push->pose':>27s}  "
          f"{'>interval FE/BE':>15s}  {'trend':>6s}")
    for path in args.paths:
        f = os.path.join(path, "timing.csv") if os.path.isdir(path) else path
        with open(f) as fh:
            header = fh.readline().lstrip("#").strip().split(",")
            idx = {n: i for i, n in enumerate(header)}
            if "frontend_frames_received" not in idx:
                print(f"{path}: no Basalt stage timestamps (run with SLAM_TIMING_STAT=1)")
                continue
            rows = [list(map(int, r)) for r in csv.reader(fh) if len(r) == len(header)]
        if not rows:
            print(f"{path}: no complete rows")
            continue
        fe = stage(rows, idx, "frontend_frames_received", "frontend_keypoints_pushed")
        be = stage(rows, idx, "backend_keypoints_received", "backend_state_pushed")
        e2e = stage(rows, idx, "frames_pushed", "backend_state_pushed")
        fmt = lambda v: f"{pct(v, 50):5.1f} / {pct(v, 95):5.1f} / {pct(v, 99):5.1f} / {max(v):6.1f}"
        over_fe = 100 * sum(x > budget for x in fe) / len(fe)
        over_be = 100 * sum(x > budget for x in be) / len(be)
        # Queue growth: mean push->pose latency of the last fifth minus the second fifth (after warm-up).
        k = len(e2e) // 5
        trend = (sum(e2e[-k:]) - sum(e2e[k:2 * k])) / k if k else 0.0
        name = os.path.basename(os.path.normpath(path if os.path.isdir(path) else os.path.dirname(path)))
        print(f"{name:30s} {len(rows):6d}  {fmt(fe)}  {fmt(be)}  {fmt(e2e)}  "
              f"{over_fe:6.1f}% /{over_be:5.1f}%  {trend:+6.1f}")


if __name__ == "__main__":
    main()
