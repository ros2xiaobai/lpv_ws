#!/usr/bin/env python3
import csv
import math
import os
from datetime import datetime


def main():
    results_dir = "/home/lxx/LPV_ws/src/lpv_attack_sim/results"
    os.makedirs(results_dir, exist_ok=True)
    path = os.path.join(results_dir, "setpoint_fdi_attack_preview_%s.csv" % datetime.now().strftime("%Y%m%d_%H%M%S"))

    dt = 1.0 / 30.0
    warmup_time = 4.0
    pre_attack_time = 18.0
    attack_duration = 35.0
    post_attack_time = 12.0
    total_time = warmup_time + pre_attack_time + attack_duration + post_attack_time
    attack_start = warmup_time + pre_attack_time
    attack_end = attack_start + attack_duration

    radius = 2.0
    angular_rate = 0.12
    z_ref = 2.0
    actual = [0.0, 0.0, 0.0]
    prev_actual = list(actual)
    tau_response = 1.2

    header = [
        "t", "phase",
        "nominal_x", "nominal_y", "nominal_z",
        "attacked_x", "attacked_y", "attacked_z",
        "attack_x", "attack_y", "attack_z",
        "actual_x", "actual_y", "actual_z",
        "vel_x", "vel_y", "vel_z",
        "roll", "pitch", "yaw",
        "err_to_nominal", "err_to_attacked",
        "mavros_mode", "armed",
    ]

    rows = []
    steps = int(total_time / dt) + 1
    for k in range(steps):
        t = k * dt
        if t < warmup_time:
            nx, ny, nz = 0.0, 0.0, z_ref
        else:
            theta = angular_rate * (t - warmup_time)
            nx = radius * (math.cos(theta) - 1.0)
            ny = radius * math.sin(theta)
            nz = z_ref

        if t < attack_start:
            phase = "baseline"
            ax, ay, az = 0.0, 0.0, 0.0
        elif t <= attack_end:
            phase = "attack"
            atk_t = t - attack_start
            ax, ay, az = 1.2 + 0.015 * atk_t, 0.8, 0.0
        else:
            phase = "post_attack"
            ax, ay, az = 0.0, 0.0, 0.0

        tx, ty, tz = nx + ax, ny + ay, nz + az
        prev_actual = list(actual)
        alpha = min(1.0, dt / tau_response)
        actual[0] += alpha * (tx - actual[0])
        actual[1] += alpha * (ty - actual[1])
        actual[2] += alpha * (tz - actual[2])

        vx = (actual[0] - prev_actual[0]) / dt
        vy = (actual[1] - prev_actual[1]) / dt
        vz = (actual[2] - prev_actual[2]) / dt
        roll = max(-0.35, min(0.35, -0.10 * vy))
        pitch = max(-0.35, min(0.35, 0.10 * vx))
        yaw = 0.0

        err_nom = math.sqrt((actual[0] - nx) ** 2 + (actual[1] - ny) ** 2 + (actual[2] - nz) ** 2)
        err_atk = math.sqrt((actual[0] - tx) ** 2 + (actual[1] - ty) ** 2 + (actual[2] - tz) ** 2)
        rows.append([
            "%.4f" % t, phase,
            "%.6f" % nx, "%.6f" % ny, "%.6f" % nz,
            "%.6f" % tx, "%.6f" % ty, "%.6f" % tz,
            "%.6f" % ax, "%.6f" % ay, "%.6f" % az,
            "%.6f" % actual[0], "%.6f" % actual[1], "%.6f" % actual[2],
            "%.6f" % vx, "%.6f" % vy, "%.6f" % vz,
            "%.6f" % roll, "%.6f" % pitch, "%.6f" % yaw,
            "%.6f" % err_nom, "%.6f" % err_atk,
            "PREVIEW", "True",
        ])

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)
    print(path)


if __name__ == "__main__":
    main()
