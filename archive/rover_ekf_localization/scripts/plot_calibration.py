#!/usr/bin/env python3
"""Before/after calibration figure from odom_diagnose.py CSVs.

Plots commanded vs scan-matched (rf2o) forward velocity over time, and
gyro vs rf2o yaw rate per sample, for the first
drive test (183827, old gains, raw gyro) and the final loop (192537).
Usage: plot_calibration.py [data_dir] [out.png]
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent.parent
DATA = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "data" / "odom_diag"
OUT = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE / "figures" / "calibration_before_after.png"
RUNS = [("20260924_183827", "Before (first drive test)"),
        ("20260924_192537", "After (final loop)")]


def load(run):
    d = np.genfromtxt(DATA / f"{run}.csv", delimiter=",", names=True)
    return {k: d[k] for k in d.dtype.names}


def fit(x, y):
    slope = np.dot(x, y) / np.dot(x, x)      # through the origin
    r = np.corrcoef(x, y)[0, 1]
    return slope, r


fig, axes = plt.subplots(2, 2, figsize=(10, 9))
for col, (run, title) in enumerate(RUNS):
    d = load(run)

    ax = axes[0, col]
    # Teleop drives at one throttle, so a cmd-vs-measured scatter is a
    # vertical line; a time series shows the gain error directly.
    t = d["t"] - d["t"][0]
    fwd = d["cmd_vx"] > 0.01
    first = np.argmax(fwd)
    w = (t >= t[first] - 3) & (t <= t[first] + 40)
    ratio = np.median(d["rf2o_vx"][fwd] / d["cmd_vx"][fwd])
    ax.plot(t[w], d["cmd_vx"][w], "k--", lw=1.5, label="commanded (model)")
    ax.plot(t[w], d["rf2o_vx"][w], lw=1, label="measured (rf2o)")
    ax.set(title=f"{title}\nforward velocity (measured/commanded = {ratio:.2f})",
           xlabel="time (s)", ylabel="vx (m/s)", ylim=(-0.05, 0.4))
    ax.legend(loc="upper right", fontsize=8)

    ax = axes[1, col]
    m = np.abs(d["rf2o_wz"]) > 0.05
    x, y = d["rf2o_wz"][m], d["gyro_wz"][m]
    s, r = fit(x, y)
    ax.scatter(x, y, s=8, alpha=0.5, color="tab:green")
    xl = np.abs(x).max() * 1.1
    ax.plot([-xl, xl], [-xl, xl], "k--", lw=1, label="ideal (1:1)")
    ax.plot([-xl, xl], [-s * xl, s * xl], "r-", lw=1.5,
            label=f"fit: slope {s:+.2f}, r {r:+.2f}")
    ax.set(title="yaw rate: gyro vs scan matching",
           xlabel="measured (rf2o) yaw rate (rad/s)",
           ylabel="gyro yaw rate (as published)")
    ax.legend(loc="upper left", fontsize=8)

fig.suptitle("Odometry input calibration, before vs after", fontsize=13)
fig.tight_layout()
OUT.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(OUT, dpi=120)
print(f"wrote {OUT}")
