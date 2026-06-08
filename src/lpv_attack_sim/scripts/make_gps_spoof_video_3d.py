#!/usr/bin/env python3
import argparse
import csv
import os
import sys
import tempfile

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from result_paths import latest_matching_file


def latest_csv(results_dir):
    try:
        return latest_matching_file(results_dir, "gps_spoof_attack_*.csv")
    except FileNotFoundError:
        raise FileNotFoundError("No gps_spoof_attack_*.csv found in %s" % results_dir)


def read_csv(path):
    data = {}
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for key in reader.fieldnames:
            data[key] = []
        for row in reader:
            for key, value in row.items():
                if key in ("phase", "mavros_mode", "armed"):
                    data[key].append(value)
                else:
                    data[key].append(float(value))
    return data


def axis_limits(*series, pad=0.8):
    vals = []
    for item in series:
        vals.extend(item)
    lo = min(vals) - pad
    hi = max(vals) + pad
    if abs(hi - lo) < 1e-6:
        hi = lo + 1.0
    return lo, hi


PHASE_STYLES = {
    "baseline": ("Baseline", "#2c3e50"),
    "attack": ("GPS spoof on", "#c0392b"),
    "hold": ("Spoof hold", "#d35400"),
    "post_attack": ("Recovery", "#7f8c8d"),
}


def phase_ranges(data):
    ranges = []
    start_idx = 0
    phases = data["phase"]
    for idx in range(1, len(phases)):
        if phases[idx] != phases[start_idx]:
            ranges.append((phases[start_idx], data["t"][start_idx], data["t"][idx - 1]))
            start_idx = idx
    ranges.append((phases[start_idx], data["t"][start_idx], data["t"][-1]))
    return ranges


def draw_phase_timeline(fig, data, ranges, idx):
    axis = fig.add_axes([0.20, 0.045, 0.62, 0.055])
    t0 = data["t"][0]
    t1 = data["t"][-1]
    for phase, start, end in ranges:
        label, color = PHASE_STYLES.get(phase, (phase, "#7f8c8d"))
        width = max(end - start, 1e-6)
        axis.broken_barh([(start, width)], (0.0, 1.0), facecolors=color, alpha=0.82)
        if width > 5.0:
            axis.text(start + width / 2.0, 0.5, label, color="white", fontsize=8,
                      ha="center", va="center", weight="bold")

    current_t = data["t"][idx]
    axis.axvline(current_t, color="black", linewidth=2.0)
    axis.set_xlim(t0, t1)
    axis.set_ylim(0.0, 1.0)
    axis.set_yticks([])
    axis.set_xlabel("simulation time / s", fontsize=8, labelpad=1)
    axis.tick_params(axis="x", labelsize=8, pad=1)
    for spine in axis.spines.values():
        spine.set_visible(False)


def main():
    parser = argparse.ArgumentParser(description="Generate a 3D trajectory MP4 video for GPS spoofing demo.")
    parser.add_argument("--csv", default=None, help="CSV result file. Defaults to latest GPS spoof result.")
    parser.add_argument("--results-dir", default="/home/lxx/LPV_ws/src/lpv_attack_sim/results")
    parser.add_argument("--output", default=None, help="Output MP4 path.")
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=28.0)
    parser.add_argument("--elev", type=float, default=20.0, help="Camera elevation angle")
    parser.add_argument("--azim", type=float, default=-60.0, help="Camera azimuth angle")
    args = parser.parse_args()

    csv_path = args.csv or latest_csv(args.results_dir)
    data = read_csv(csv_path)
    output = args.output or os.path.splitext(csv_path)[0] + "_3d_deviation.mp4"

    n = len(data["t"])
    if n < 2:
        raise RuntimeError("CSV contains too few samples.")

    frame_count = max(2, int(args.fps * args.duration))
    indices = [int(round(i * (n - 1) / (frame_count - 1))) for i in range(frame_count)]
    ranges = phase_ranges(data)

    xlim = axis_limits(data["nominal_x"], data["actual_x"])
    ylim = axis_limits(data["nominal_y"], data["actual_y"])
    zlim = axis_limits(data["nominal_z"], data["actual_z"])

    width, height = 1280, 720
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(output, fourcc, args.fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError("Failed to open video writer for %s" % output)

    with tempfile.TemporaryDirectory() as tmpdir:
        for frame_id, idx in enumerate(indices):
            fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
            ax = fig.add_subplot(111, projection='3d')

            # Nominal reference trajectory (full circle)
            ax.plot(data["nominal_x"], data["nominal_y"], data["nominal_z"],
                    color="#3498db", linewidth=2.5, label="Nominal reference", alpha=0.7)

            # PX4 estimate after GPS fusion (what the controller believes).
            ax.plot(data["estimated_x"][:idx + 1], data["estimated_y"][:idx + 1], data["estimated_z"][:idx + 1],
                    "--", color="#e67e22", linewidth=2.2, label="PX4 estimated position", alpha=0.9)

            # Actual trajectory (true path, grows with time)
            ax.plot(data["actual_x"][:idx + 1], data["actual_y"][:idx + 1], data["actual_z"][:idx + 1],
                    color="#27ae60", linewidth=2.8, label="Actual trajectory", alpha=1.0)

            # Current positions
            ax.scatter(data["actual_x"][idx], data["actual_y"][idx], data["actual_z"][idx],
                       color="#27ae60", s=80, marker='o', edgecolors='black', linewidths=1.5)
            ax.scatter(data["estimated_x"][idx], data["estimated_y"][idx], data["estimated_z"][idx],
                       color="#e67e22", s=60, marker='x', linewidths=2.5)

            ax.set_xlim(*xlim)
            ax.set_ylim(*ylim)
            ax.set_zlim(*zlim)
            ax.set_xlabel("x / m", fontsize=11)
            ax.set_ylabel("y / m", fontsize=11)
            ax.set_zlabel("z / m", fontsize=11)
            ax.set_title("Trajectory deviation under GPS spoofing attack", fontsize=14, pad=15)
            ax.legend(loc="upper left", fontsize=10)

            # Set viewing angle
            ax.view_init(elev=args.elev, azim=args.azim)

            # Add time and phase status
            phase = data["phase"][idx]
            if phase == "attack":
                status = "GPS SPOOF ON"
                color = "#c0392b"
            elif phase == "hold":
                status = "GPS SPOOF HOLD"
                color = "#d35400"
            else:
                status = "NO SPOOF"
                color = "#2c3e50"
            fig.text(0.02, 0.02, "t = %.1f s  |  %s" % (data["t"][idx], status),
                     fontsize=13, color=color, weight='bold',
                     bbox=dict(boxstyle="round,pad=0.5", facecolor="white", edgecolor=color, alpha=0.95))

            fig.tight_layout(rect=[0.0, 0.10, 1.0, 1.0])
            draw_phase_timeline(fig, data, ranges, idx)
            png_path = os.path.join(tmpdir, "frame_%05d.png" % frame_id)
            fig.savefig(png_path)
            plt.close(fig)

            frame = cv2.imread(png_path)
            if frame is None:
                raise RuntimeError("Failed to read generated frame.")
            writer.write(frame)

    writer.release()
    print(output)


if __name__ == "__main__":
    main()
