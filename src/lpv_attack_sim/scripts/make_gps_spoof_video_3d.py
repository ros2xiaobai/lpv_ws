#!/usr/bin/env python3
import argparse
import csv
import glob
import os
import tempfile

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def latest_csv(results_dir):
    files = sorted(glob.glob(os.path.join(results_dir, "gps_spoof_attack_*.csv")))
    if not files:
        raise FileNotFoundError("No gps_spoof_attack_*.csv found in %s" % results_dir)
    return files[-1]


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

            # Spoofed localization input (what PX4 thinks, grows with time)
            ax.plot(data["estimated_x"][:idx + 1], data["estimated_y"][:idx + 1], data["estimated_z"][:idx + 1],
                    "--", color="#e67e22", linewidth=2.2, label="Attacked setpoint", alpha=0.9)

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
            status = "GPS SPOOF ON" if phase == "attack" else "NO SPOOF"
            color = "#c0392b" if phase == "attack" else "#2c3e50"
            fig.text(0.02, 0.02, "t = %.1f s  |  %s" % (data["t"][idx], status),
                     fontsize=13, color=color, weight='bold',
                     bbox=dict(boxstyle="round,pad=0.5", facecolor="white", edgecolor=color, alpha=0.95))

            fig.tight_layout()
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
