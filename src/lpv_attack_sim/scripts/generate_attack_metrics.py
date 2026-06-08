#!/usr/bin/env python3
"""
Generate quantitative metrics from GPS spoofing attack CSV logs.
"""
import argparse
import csv
import json
import math
import os
import sys
import numpy as np


def read_csv(path):
    """Read CSV and return list of dicts."""
    data = []
    with open(path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Convert numeric fields
            converted = {'phase': row['phase'], 'mavros_mode': row['mavros_mode'], 'armed': row['armed']}
            for key, value in row.items():
                if key not in converted:
                    try:
                        converted[key] = float(value)
                    except ValueError:
                        converted[key] = value
            data.append(converted)
    return data


def compute_metrics(data):
    """Compute quantitative attack evaluation metrics."""
    # Split by phase
    baseline = [r for r in data if r['phase'] == 'baseline']
    attack = [r for r in data if r['phase'] == 'attack']
    hold = [r for r in data if r['phase'] == 'hold']
    post_attack = [r for r in data if r['phase'] == 'post_attack']

    if not attack:
        return {'error': 'No attack phase found in data'}

    metrics = {}

    # === Baseline performance ===
    if baseline:
        metrics['baseline'] = {
            'duration_s': baseline[-1]['t'] - baseline[0]['t'],
            'mean_tracking_error_m': np.mean([r['err_to_nominal'] for r in baseline]),
            'max_tracking_error_m': np.max([r['err_to_nominal'] for r in baseline]),
            'std_tracking_error_m': np.std([r['err_to_nominal'] for r in baseline]),
        }
    else:
        metrics['baseline'] = {'duration_s': 0.0}

    # === Attack phase analysis ===
    attack_errors = [r['err_to_nominal'] for r in attack]
    attack_velocities = [math.hypot(r['vel_x'], r['vel_y']) for r in attack]

    metrics['attack'] = {
        'duration_s': attack[-1]['t'] - attack[0]['t'],
        'start_time_s': attack[0]['t'],
        'end_time_s': attack[-1]['t'],

        # Deviation metrics
        'max_deviation_m': np.max(attack_errors),
        'mean_deviation_m': np.mean(attack_errors),
        'final_deviation_m': attack_errors[-1],
        'std_deviation_m': np.std(attack_errors),

        # Velocity metrics
        'mean_velocity_m_s': np.mean(attack_velocities),
        'max_velocity_m_s': np.max(attack_velocities),
    }

    # Compute velocity change rate (acceleration proxy)
    if len(attack) > 1:
        velocity_changes = []
        for i in range(1, len(attack)):
            dt = attack[i]['t'] - attack[i-1]['t']
            if dt > 1e-6:
                v1 = math.hypot(attack[i-1]['vel_x'], attack[i-1]['vel_y'])
                v2 = math.hypot(attack[i]['vel_x'], attack[i]['vel_y'])
                dv = abs(v2 - v1)
                velocity_changes.append(dv / dt)

        if velocity_changes:
            metrics['attack']['max_acceleration_m_s2'] = np.max(velocity_changes)
            metrics['attack']['mean_acceleration_m_s2'] = np.mean(velocity_changes)

    # === Persistent spoof hold analysis ===
    if hold:
        hold_errors = [r['err_to_nominal'] for r in hold]
        hold_estimated_errors = [r.get('estimated_err_to_nominal', 0.0) for r in hold]
        hold_estimate_gaps = [r.get('estimate_actual_gap', 0.0) for r in hold]
        metrics['hold'] = {
            'duration_s': hold[-1]['t'] - hold[0]['t'],
            'initial_deviation_m': hold_errors[0],
            'final_deviation_m': hold_errors[-1],
            'mean_deviation_m': np.mean(hold_errors),
            'min_deviation_m': np.min(hold_errors),
            'max_deviation_m': np.max(hold_errors),
            'mean_estimated_error_m': np.mean(hold_estimated_errors),
            'final_estimated_error_m': hold_estimated_errors[-1],
            'mean_estimate_actual_gap_m': np.mean(hold_estimate_gaps),
            'final_estimate_actual_gap_m': hold_estimate_gaps[-1],
        }

    # === Recovery phase analysis ===
    if post_attack:
        post_errors = [r['err_to_nominal'] for r in post_attack]

        # Find recovery time (time to get back within threshold)
        recovery_threshold_m = 0.5
        recovery_time = None
        for r in post_attack:
            if r['err_to_nominal'] < recovery_threshold_m:
                recovery_time = r['t'] - attack[-1]['t']
                break

        metrics['post_attack'] = {
            'duration_s': post_attack[-1]['t'] - post_attack[0]['t'],
            'initial_error_m': post_errors[0],
            'final_error_m': post_errors[-1],
            'recovery_time_s': recovery_time if recovery_time else None,
            'recovery_threshold_m': recovery_threshold_m,
        }

        # Compute recovery rate
        if len(post_attack) > 1:
            post_velocities = []
            for i in range(1, len(post_attack)):
                dt = post_attack[i]['t'] - post_attack[i-1]['t']
                if dt > 1e-6:
                    v1 = math.hypot(post_attack[i-1]['vel_x'], post_attack[i-1]['vel_y'])
                    v2 = math.hypot(post_attack[i]['vel_x'], post_attack[i]['vel_y'])
                    dv = abs(v2 - v1)
                    post_velocities.append(dv / dt)

            if post_velocities:
                metrics['post_attack']['max_acceleration_m_s2'] = np.max(post_velocities)

    # === Overall summary ===
    metrics['summary'] = {
        'total_duration_s': data[-1]['t'] - data[0]['t'],
        'total_samples': len(data),
        'attack_effectiveness': metrics['attack']['max_deviation_m'] / (metrics['baseline']['mean_tracking_error_m'] + 1e-6) if baseline else None,
        'persistent_hold': bool(hold),
    }

    # === Spoofing success criteria ===
    # A successful GPS spoof should keep the real trajectory displaced while
    # PX4's fused estimate still appears close to the nominal reference.
    success_thresholds = {
        'min_hold_duration_s': 5.0,
        'min_actual_deviation_m': 2.0,
        'max_estimated_error_m': 0.5,
        'min_estimate_actual_gap_m': 2.0,
    }
    if hold:
        h = metrics['hold']
        checks = {
            'persistent_hold_available': True,
            'hold_duration_ok': bool(h['duration_s'] >= success_thresholds['min_hold_duration_s']),
            'actual_deviation_ok': bool(h['mean_deviation_m'] >= success_thresholds['min_actual_deviation_m']),
            'estimated_tracks_nominal_ok': bool(h['mean_estimated_error_m'] <= success_thresholds['max_estimated_error_m']),
            'estimate_truth_gap_ok': bool(h['mean_estimate_actual_gap_m'] >= success_thresholds['min_estimate_actual_gap_m']),
        }
    else:
        checks = {
            'persistent_hold_available': False,
            'hold_duration_ok': False,
            'actual_deviation_ok': False,
            'estimated_tracks_nominal_ok': False,
            'estimate_truth_gap_ok': False,
        }

    metrics['spoof_success'] = {
        'passed': all(checks.values()),
        'thresholds': success_thresholds,
        'checks': checks,
    }

    return metrics


def format_metrics_text(metrics):
    """Format metrics as human-readable text."""
    lines = []
    lines.append("=" * 70)
    lines.append("GPS SPOOFING ATTACK QUANTITATIVE METRICS")
    lines.append("=" * 70)

    if 'baseline' in metrics and metrics['baseline']['duration_s'] > 0:
        b = metrics['baseline']
        lines.append("\n[ Baseline Phase ]")
        lines.append(f"  Duration:              {b['duration_s']:.2f} s")
        lines.append(f"  Mean tracking error:   {b['mean_tracking_error_m']:.3f} m")
        lines.append(f"  Max tracking error:    {b['max_tracking_error_m']:.3f} m")
        lines.append(f"  Std tracking error:    {b['std_tracking_error_m']:.3f} m")

    if 'attack' in metrics:
        a = metrics['attack']
        lines.append("\n[ Attack Phase ]")
        lines.append(f"  Duration:              {a['duration_s']:.2f} s  ({a['start_time_s']:.1f}s - {a['end_time_s']:.1f}s)")
        lines.append(f"  Max deviation:         {a['max_deviation_m']:.3f} m")
        lines.append(f"  Mean deviation:        {a['mean_deviation_m']:.3f} m")
        lines.append(f"  Final deviation:       {a['final_deviation_m']:.3f} m")
        lines.append(f"  Std deviation:         {a['std_deviation_m']:.3f} m")
        if 'max_acceleration_m_s2' in a:
            lines.append(f"  Max acceleration:      {a['max_acceleration_m_s2']:.3f} m/s²")
        lines.append(f"  Mean velocity:         {a['mean_velocity_m_s']:.3f} m/s")

    if 'hold' in metrics:
        h = metrics['hold']
        lines.append("\n[ Post-Attack Spoof Hold ]")
        lines.append(f"  Duration:              {h['duration_s']:.2f} s")
        lines.append(f"  Initial deviation:     {h['initial_deviation_m']:.3f} m")
        lines.append(f"  Final deviation:       {h['final_deviation_m']:.3f} m")
        lines.append(f"  Mean deviation:        {h['mean_deviation_m']:.3f} m")
        lines.append(f"  Min/Max deviation:     {h['min_deviation_m']:.3f} / {h['max_deviation_m']:.3f} m")
        lines.append(f"  Mean estimate error:   {h['mean_estimated_error_m']:.3f} m")
        lines.append(f"  Mean EKF truth gap:    {h['mean_estimate_actual_gap_m']:.3f} m")

    if 'spoof_success' in metrics:
        ss = metrics['spoof_success']
        checks = ss['checks']
        lines.append("\n[ Spoofing Success Check ]")
        lines.append(f"  Result:                {'PASS' if ss['passed'] else 'FAIL'}")
        lines.append(f"  Persistent hold:       {checks['persistent_hold_available']}")
        lines.append(f"  Hold duration OK:      {checks['hold_duration_ok']}")
        lines.append(f"  Actual deviation OK:   {checks['actual_deviation_ok']}")
        lines.append(f"  Estimate near nominal: {checks['estimated_tracks_nominal_ok']}")
        lines.append(f"  Estimate/truth split:  {checks['estimate_truth_gap_ok']}")

    if 'post_attack' in metrics:
        p = metrics['post_attack']
        lines.append("\n[ Post-Attack Recovery ]")
        lines.append(f"  Duration:              {p['duration_s']:.2f} s")
        lines.append(f"  Initial error:         {p['initial_error_m']:.3f} m")
        lines.append(f"  Final error:           {p['final_error_m']:.3f} m")
        if p['recovery_time_s']:
            lines.append(f"  Recovery time:         {p['recovery_time_s']:.2f} s  (to < {p['recovery_threshold_m']}m)")
        else:
            lines.append(f"  Recovery time:         Not achieved within observation period")
        if 'max_acceleration_m_s2' in p:
            lines.append(f"  Max acceleration:      {p['max_acceleration_m_s2']:.3f} m/s²")

    if 'summary' in metrics:
        s = metrics['summary']
        lines.append("\n[ Overall Summary ]")
        lines.append(f"  Total duration:        {s['total_duration_s']:.2f} s")
        lines.append(f"  Total samples:         {s['total_samples']}")
        if s['attack_effectiveness']:
            lines.append(f"  Attack effectiveness:  {s['attack_effectiveness']:.1f}x baseline error")
        lines.append(f"  Persistent hold:       {s['persistent_hold']}")

    lines.append("\n" + "=" * 70)
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Generate quantitative metrics from GPS spoofing attack CSV."
    )
    parser.add_argument(
        'csv_path',
        help='Path to GPS spoof attack CSV file'
    )
    parser.add_argument(
        '--output-json',
        help='Output path for JSON metrics (default: <csv>_metrics.json)'
    )
    parser.add_argument(
        '--output-txt',
        help='Output path for text report (default: <csv>_metrics.txt)'
    )
    args = parser.parse_args()

    if not os.path.exists(args.csv_path):
        print(f"Error: CSV file not found: {args.csv_path}", file=sys.stderr)
        sys.exit(1)

    # Default output paths
    base = os.path.splitext(args.csv_path)[0]
    json_path = args.output_json or f"{base}_metrics.json"
    txt_path = args.output_txt or f"{base}_metrics.txt"

    try:
        data = read_csv(args.csv_path)
        metrics = compute_metrics(data)

        # Write JSON
        with open(json_path, 'w') as f:
            json.dump(metrics, f, indent=2)
        print(f"[Metrics] JSON saved to: {json_path}")

        # Write text report
        text_report = format_metrics_text(metrics)
        with open(txt_path, 'w') as f:
            f.write(text_report)
        print(f"[Metrics] Text report saved to: {txt_path}")

        # Print to console
        print("\n" + text_report)

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
