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


def latest_csv(results_dir):
    files = sorted(glob.glob(os.path.join(results_dir, "true_gps_input_attack_*.csv")))
    if not files:
        raise FileNotFoundError("No true_gps_input_attack_*.csv found in %s" % results_dir)
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
    parser = argparse.ArgumentParser(description="Generate an MP4 for the true GPS input spoofing demo.")
    parser.add_argument("--csv", default=None, help="CSV result file. Defaults to latest true GPS input result.")
    parser.add_argument("--results-dir", default="/home/lxx/LPV_ws/src/lpv_attack_sim/results")
    parser.add_argument("--output", default=None, help="Output MP4 path.")
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=28.0)
    args = parser.parse_args()

    csv_path = args.csv or latest_csv(args.results_dir)
    data = read_csv(csv_path)
    output = args.output or os.path.splitext(csv_path)[0] + "_csv_plot.mp4"

    n = len(data["t"])
    if n < 2:
        raise RuntimeError("CSV contains too few samples.")

    frame_count = max(2, int(args.fps * args.duration))
    indices = [int(round(i * (n - 1) / (frame_count - 1))) for i in range(frame_count)]

    xlim = axis_limits(data["nominal_x"], data["gps_spoofed_x"], data["actual_x"], data["estimated_x"])
    ylim = axis_limits(data["nominal_y"], data["gps_spoofed_y"], data["actual_y"], data["estimated_y"])
    tmax = data["t"][-1]
    err_max = max(max(data["err_actual_to_nominal"]), max(data["estimate_actual_gap"]), 0.5)
    spoof_max = max(max(abs(v) for v in data["attack_x"]),
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
            ax_spoof = fig.add_subplot(grid[0, 1])
            ax_error = fig.add_subplot(grid[1, 1])

            ax_traj.plot(data["nominal_x"], data["nominal_y"], color="#7f8c8d", linewidth=1.5, label="Nominal reference")
            ax_traj.plot(data["gps_spoofed_x"][:idx + 1], data["gps_spoofed_y"][:idx + 1],
                         "--", color="#d35400", linewidth=2.0, label="Injected GPS position")
            ax_traj.plot(data["estimated_x"][:idx + 1], data["estimated_y"][:idx + 1],
                         color="#27ae60", linewidth=1.8, label="PX4 estimated trajectory")
            ax_traj.plot(data["actual_x"][:idx + 1], data["actual_y"][:idx + 1],
                         color="#1f77b4", linewidth=2.4, label="Gazebo actual trajectory")
            ax_traj.scatter(data["actual_x"][idx], data["actual_y"][idx], color="#1f77b4", s=55)
            ax_traj.scatter(data["gps_spoofed_x"][idx], data["gps_spoofed_y"][idx], color="#d35400", s=45, marker="x")
            ax_traj.scatter(data["estimated_x"][idx], data["estimated_y"][idx], color="#27ae60", s=38)
            ax_traj.set_xlim(*xlim)
            ax_traj.set_ylim(*ylim)
            ax_traj.set_aspect("equal", adjustable="box")
            ax_traj.grid(True, alpha=0.35)
            ax_traj.set_xlabel("x / m")
            ax_traj.set_ylabel("y / m")
            ax_traj.set_title("Trajectory deviation under true GPS input spoofing")
            ax_traj.legend(loc="upper left")

            phase = data["phase"][idx]
            status = "GPS INPUT SPOOF ON" if phase == "attack" else "NO SPOOF"
            color = "#c0392b" if phase == "attack" else "#2c3e50"
            ax_traj.text(0.02, 0.02, "t = %.1f s  |  %s" % (data["t"][idx], status),
                         transform=ax_traj.transAxes, fontsize=12, color=color,
                         bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor=color, alpha=0.9))

            ax_spoof.plot(data["t"][:idx + 1], data["attack_x"][:idx + 1], label="gps_bias_x", color="#c0392b")
            ax_spoof.plot(data["t"][:idx + 1], data["attack_y"][:idx + 1], label="gps_bias_y", color="#8e44ad")
            ax_spoof.plot(data["t"][:idx + 1], data["attack_z"][:idx + 1], label="gps_bias_z", color="#16a085")
            ax_spoof.set_xlim(0.0, tmax)
            ax_spoof.set_ylim(-0.2, spoof_max + 0.35)
            ax_spoof.set_ylabel("GPS bias / m")
            ax_spoof.set_title("Injected GPS sensor bias")
            ax_spoof.grid(True, alpha=0.35)
            ax_spoof.legend(loc="upper left", ncol=3)

            ax_error.plot(data["t"][:idx + 1], data["err_actual_to_nominal"][:idx + 1],
                          color="#2980b9", label="actual error to nominal")
            ax_error.plot(data["t"][:idx + 1], data["estimate_actual_gap"][:idx + 1],
                          color="#d35400", label="estimated-actual gap")
            ax_error.plot(data["t"][:idx + 1], data["err_estimated_to_nominal"][:idx + 1],
                          color="#27ae60", label="estimated error to nominal")
            ax_error.set_xlim(0.0, tmax)
            ax_error.set_ylim(0.0, err_max + 0.35)
            ax_error.set_xlabel("time / s")
            ax_error.set_ylabel("error / m")
            ax_error.set_title("GPS deception and tracking errors")
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
