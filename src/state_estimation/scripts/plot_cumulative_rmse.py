#!/usr/bin/env python3
"""
Plot cumulative RMSE (xy + orientation) over time for a set of filter test
runs, colored using the palette defined in filter_color_palette.py.

Expects one CSV per test run with columns: t, cum_rmse_xy, cum_rmse_theta
(theta in radians) — this is exactly what the CSV-logging patch added to
error_handler.py's FilterTracker writes out (see chat for that patch).

Edit the RUNS dict below to point at your actual log files.
"""

import csv
import math
import os
import sys

import matplotlib.pyplot as plt

# So `import filter_color_palette` works regardless of the current working
# directory, as long as both files sit in the same folder (e.g. scripts/).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from filter_colour_palette import FILTER_PALETTES


# Each entry: csv_path -> (family, shade_index, legend_label)
#   - family must be a key in FILTER_PALETTES (KF / EKF_landmark /
#     EKF_no_landmark / PF)
#   - shade_index picks which shade within that family's palette
# ADJUST this to point at your actual log files — add/remove rows freely.
RUNS = {
    'rmse_logs/rmse_log_KF.csv':        ('KF', 0, 'KF — baseline'),
    'rmse_logs/rmse_log_KF_tuned.csv':           ('KF', 1, 'KF — tuned'),
    'rmse_logs/rmse_log_EKFI.csv':    ('EKF_landmark', 0, 'EKF (landmark)'),
    'rmse_logs/rmse_log_EKF_landmark_v2.csv':    ('EKF_landmark', 1, 'EKF (landmark) v2'),
    'rmse_logs/rmse_log_EKF_landmark_v3.csv':    ('EKF_landmark', 2, 'EKF (landmark) v3'),
    'rmse_logs/rmse_log_EKF_landmark_v4.csv':    ('EKF_landmark', 3, 'EKF (landmark) v4'),
    'rmse_logs/rmse_log_EKFO.csv': ('EKF_no_landmark', 0, 'EKF (odom only)'),
    'rmse_logs/rmse_log_EKF_no_landmark_v2.csv': ('EKF_no_landmark', 1, 'EKF (no landmark) v2'),
    'rmse_logs/rmse_log_PF.csv':                 ('PF', 0, 'PF'),
}

OUTPUT_PATH = 'cumulative_rmse.png'  # change extension to .pdf/.svg for vector output


def load_run(csv_path):
    t, xy, theta = [], [], []
    with open(csv_path, newline='') as f:
        for row in csv.DictReader(f):
            t.append(float(row['t']))
            xy.append(float(row['cum_rmse_xy']))
            theta.append(math.degrees(float(row['cum_rmse_theta'])))
    return t, xy, theta


def main():
    fig, (ax_xy, ax_theta) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    for csv_path, (family, shade_idx, label) in RUNS.items():
        if not os.path.exists(csv_path):
            print(f"skipping (not found): {csv_path}")
            continue

        palette = FILTER_PALETTES.get(family)
        if palette is None:
            print(f"skipping '{label}': unknown family '{family}' "
                  f"(check it matches a key in FILTER_PALETTES)")
            continue
        color = palette[min(shade_idx, len(palette) - 1)]

        t, xy, theta = load_run(csv_path)
        if not t:
            print(f"skipping '{label}': {csv_path} has no rows")
            continue

        ax_xy.plot(t, xy, color=color, label=label)
        ax_theta.plot(t, theta, color=color, label=label)

    ax_xy.set_ylabel('Cumulative RMSE — xy (m)')
    ax_theta.set_ylabel('Cumulative RMSE — orientation (deg)')
    ax_theta.set_xlabel('Time (s)')
    ax_xy.grid(alpha=0.3)
    ax_theta.grid(alpha=0.3)

    handles, labels = ax_xy.get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc='center left', bbox_to_anchor=(1.0, 0.5),
                   fontsize=8, frameon=False)

    fig.suptitle('Cumulative RMSE over test trajectory')
    fig.tight_layout(rect=[0, 0, 0.82, 1])  # leave room for the legend
    fig.savefig(OUTPUT_PATH, dpi=150, bbox_inches='tight')
    print(f"Saved {OUTPUT_PATH}")


if __name__ == '__main__':
    main()
