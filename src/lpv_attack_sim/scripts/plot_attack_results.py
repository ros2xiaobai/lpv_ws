#!/usr/bin/env python3
import argparse
import csv
import glob
import os

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def latest_csv(results_dir):
    # Try GPS spoof attack files first, then fall back to setpoint FDI attack files
    files = sorted(glob.glob(os.path.join(results_dir, "gps_spoof_attack_*.csv")))
    if not files:
        files = sorted(glob.glob(os.path.join(results_dir, "setpoint_fdi_attack_*.csv")))
    if not files:
        files = sorted(glob.glob(os.path.join(results_dir, "true_gps_input_attack_*.csv")))
    if not files:
        raise FileNotFoundError("No attack result CSV found in %s" % results_dir)
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


def main():
    parser = argparse.ArgumentParser(description="Plot LPV FDI attack trajectory-deviation results.")
    parser.add_argument("--csv", default=None, help="CSV result file. Defaults to latest result.")
    parser.add_argument("--results-dir", default="/home/lxx/LPV_ws/src/lpv_attack_sim/results")
    args = parser.parse_args()

    csv_path = args.csv or latest_csv(args.results_dir)
    data = read_csv(csv_path)
    out_prefix = os.path.splitext(csv_path)[0]

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(data["nominal_x"], data["nominal_y"], data["nominal_z"], label="Nominal reference", linewidth=2)
    ax.plot(data["attacked_x"], data["attacked_y"], data["attacked_z"], label="Attacked setpoint", linestyle="--")
    ax.plot(data["actual_x"], data["actual_y"], data["actual_z"], label="Actual trajectory")
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.set_zlabel("z / m")
    ax.legend()
    ax.set_title("Trajectory deviation under setpoint FDI attack")
    fig.tight_layout()
    fig.savefig(out_prefix + "_trajectory_3d.png", dpi=180)

    fig2, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax_i, key, label in zip(axes, ["x", "y", "z"], ["x / m", "y / m", "z / m"]):
        ax_i.plot(data["t"], data["nominal_" + key], label="Nominal")
        ax_i.plot(data["t"], data["attacked_" + key], label="Attacked", linestyle="--")
        ax_i.plot(data["t"], data["actual_" + key], label="Actual")
        ax_i.set_ylabel(label)
        ax_i.grid(True)
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("time / s")
    fig2.suptitle("Position response")
    fig2.tight_layout()
    fig2.savefig(out_prefix + "_position.png", dpi=180)

    fig3, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot(data["t"], data["attack_x"], label="attack x")
    axes[0].plot(data["t"], data["attack_y"], label="attack y")
    axes[0].plot(data["t"], data["attack_z"], label="attack z")
    axes[0].set_ylabel("attack / m")
    axes[0].legend(loc="best")
    axes[0].grid(True)
    axes[1].plot(data["t"], data["err_to_nominal"], label="error to nominal")
    axes[1].plot(data["t"], data["err_to_attacked"], label="error to attacked")
    axes[1].set_ylabel("error / m")
    axes[1].set_xlabel("time / s")
    axes[1].legend(loc="best")
    axes[1].grid(True)
    fig3.suptitle("Attack signal and trajectory error")
    fig3.tight_layout()
    fig3.savefig(out_prefix + "_attack_error.png", dpi=180)

    print("CSV:", csv_path)
    print("Saved:")
    print("  " + out_prefix + "_trajectory_3d.png")
    print("  " + out_prefix + "_position.png")
    print("  " + out_prefix + "_attack_error.png")


if __name__ == "__main__":
    main()
