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

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from result_paths import latest_matching_file


def latest_csv(results_dir):
    try:
        return latest_matching_file(results_dir, "setpoint_fdi_attack_*.csv")
    except FileNotFoundError:
        raise FileNotFoundError("No setpoint_fdi_attack_*.csv found in %s" % results_dir)


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
    parser = argparse.ArgumentParser(description="Generate an MP4 video for the LPV FDI attack demo.")
    parser.add_argument("--csv", default=None, help="CSV result file. Defaults to latest result.")
    parser.add_argument("--results-dir", default="/home/lxx/LPV_ws/src/lpv_attack_sim/results")
    parser.add_argument("--output", default=None, help="Output MP4 path.")
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=24.0, help="Video duration in seconds.")
    args = parser.parse_args()

    csv_path = args.csv or latest_csv(args.results_dir)
    data = read_csv(csv_path)
    output = args.output or os.path.splitext(csv_path)[0] + "_trajectory_deviation.mp4"

    n = len(data["t"])
    if n < 2:
        raise RuntimeError("CSV contains too few samples.")

    frame_count = max(2, int(args.fps * args.duration))
    indices = [int(round(i * (n - 1) / (frame_count - 1))) for i in range(frame_count)]

    xlim = axis_limits(data["nominal_x"], data["attacked_x"], data["actual_x"])
    ylim = axis_limits(data["nominal_y"], data["attacked_y"], data["actual_y"])
    tmax = data["t"][-1]
    err_max = max(max(data["err_to_nominal"]), max(data["err_to_attacked"]), 0.5)
    atk_max = max(max(abs(v) for v in data["attack_x"]),
                  max(abs(v) for v in data["attack_y"]),
                  max(abs(v) for v in data["attack_z"]), 0.5)

    width, height = 1280, 720
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(output, fourcc, args.fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError("Failed to open video writer for %s" % output)

    with tempfile.TemporaryDirectory() as tmpdir:
        for frame_id, idx in enumerate(indices):
            fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
            grid = fig.add_gridspec(2, 2, width_ratios=[1.35, 1.0], height_ratios=[1.0, 1.0])

            ax_traj = fig.add_subplot(grid[:, 0])
            ax_attack = fig.add_subplot(grid[0, 1])
            ax_error = fig.add_subplot(grid[1, 1])

            ax_traj.plot(data["nominal_x"], data["nominal_y"], color="#7f8c8d", linewidth=1.5, label="Nominal reference")
            ax_traj.plot(data["attacked_x"][:idx + 1], data["attacked_y"][:idx + 1], "--", color="#d35400", linewidth=2.0, label="Attacked setpoint")
            ax_traj.plot(data["actual_x"][:idx + 1], data["actual_y"][:idx + 1], color="#1f77b4", linewidth=2.4, label="Actual trajectory")
            ax_traj.scatter(data["actual_x"][idx], data["actual_y"][idx], color="#1f77b4", s=55)
            ax_traj.scatter(data["attacked_x"][idx], data["attacked_y"][idx], color="#d35400", s=45, marker="x")
            ax_traj.set_xlim(*xlim)
            ax_traj.set_ylim(*ylim)
            ax_traj.set_aspect("equal", adjustable="box")
            ax_traj.grid(True, alpha=0.35)
            ax_traj.set_xlabel("x / m")
            ax_traj.set_ylabel("y / m")
            ax_traj.set_title("Single UAV trajectory deviation under FDI attack")
            ax_traj.legend(loc="upper left")

            phase = data["phase"][idx]
            status = "ATTACK ON" if phase == "attack" else "NO ATTACK"
            color = "#c0392b" if phase == "attack" else "#2c3e50"
            ax_traj.text(0.02, 0.02, "t = %.1f s  |  %s" % (data["t"][idx], status),
                         transform=ax_traj.transAxes, fontsize=12, color=color,
                         bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor=color, alpha=0.9))

            ax_attack.plot(data["t"][:idx + 1], data["attack_x"][:idx + 1], label="a_x", color="#c0392b")
            ax_attack.plot(data["t"][:idx + 1], data["attack_y"][:idx + 1], label="a_y", color="#8e44ad")
            ax_attack.plot(data["t"][:idx + 1], data["attack_z"][:idx + 1], label="a_z", color="#16a085")
            ax_attack.set_xlim(0.0, tmax)
            ax_attack.set_ylim(-0.2, atk_max + 0.35)
            ax_attack.set_ylabel("attack / m")
            ax_attack.set_title("Injected command FDI signal")
            ax_attack.grid(True, alpha=0.35)
            ax_attack.legend(loc="upper left", ncol=3)

            ax_error.plot(data["t"][:idx + 1], data["err_to_nominal"][:idx + 1], color="#2980b9", label="error to nominal")
            ax_error.plot(data["t"][:idx + 1], data["err_to_attacked"][:idx + 1], color="#d35400", label="error to attacked")
            ax_error.set_xlim(0.0, tmax)
            ax_error.set_ylim(0.0, err_max + 0.35)
            ax_error.set_xlabel("time / s")
            ax_error.set_ylabel("error / m")
            ax_error.set_title("Trajectory tracking error")
            ax_error.grid(True, alpha=0.35)
            ax_error.legend(loc="upper left")

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
