#!/bin/bash
# Batch run multiple GPS spoofing attack scenarios and generate comparison report

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULTS_DIR="/home/lxx/LPV_ws/src/lpv_attack_sim/results"
CONFIG_UPDATER="$SCRIPT_DIR/update_gps_spoof_config.py"
METRICS_GEN="$SCRIPT_DIR/generate_attack_metrics.py"

# Scenarios to test
SCENARIOS=("mild" "moderate" "severe")

echo "========================================================================"
echo "GPS Spoofing Attack - Batch Scenario Comparison"
echo "========================================================================"
echo ""

# Check if ROS environment is ready
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS environment not sourced. Please run:"
    echo "  source /home/lxx/LPV_ws/devel/setup.bash"
    exit 1
fi

# Clean up any running processes
echo "[Setup] Cleaning up existing processes..."
pkill -9 -f "gzserver|gzclient|px4|mavros|roslaunch|rosmaster" 2>/dev/null || true
sleep 3

# Array to store result CSV paths
RESULT_CSVS=()

# Run each scenario
for scenario in "${SCENARIOS[@]}"; do
    echo ""
    echo "========================================================================"
    echo "Running scenario: $scenario"
    echo "========================================================================"

    # Update configuration
    echo "[Step 1/4] Updating SDF configuration for scenario: $scenario"
    python3 "$CONFIG_UPDATER" --scenario "$scenario"

    # Launch simulation
    echo "[Step 2/4] Launching simulation..."
    cd /home/lxx/LPV_ws
    timeout 90s roslaunch lpv_attack_sim gps_spoof_deviation.launch > /tmp/sim_${scenario}.log 2>&1 || true

    # Find the latest CSV
    LATEST_CSV=$(ls -t "$RESULTS_DIR"/gps_spoof_attack_*.csv 2>/dev/null | head -1)

    if [ -z "$LATEST_CSV" ] || [ ! -f "$LATEST_CSV" ]; then
        echo "[Warning] No CSV generated for scenario $scenario, skipping..."
        continue
    fi

    # Rename to include scenario name
    SCENARIO_CSV="${LATEST_CSV%.csv}_${scenario}.csv"
    mv "$LATEST_CSV" "$SCENARIO_CSV"
    echo "[Step 3/4] CSV saved: $SCENARIO_CSV"

    RESULT_CSVS+=("$SCENARIO_CSV")

    # Generate metrics
    echo "[Step 4/4] Generating metrics..."
    python3 "$METRICS_GEN" "$SCENARIO_CSV"

    # Generate 3D video
    echo "[Step 4/4] Generating 3D video..."
    python3 "$SCRIPT_DIR/make_gps_spoof_video_3d.py" --csv "$SCENARIO_CSV" --output "${SCENARIO_CSV%.csv}_3d.mp4"

    # Clean up for next run
    pkill -9 -f "gzserver|gzclient|px4|mavros|roslaunch|rosmaster" 2>/dev/null || true
    sleep 5
done

echo ""
echo "========================================================================"
echo "All scenarios completed!"
echo "========================================================================"
echo ""
echo "Results:"
for csv in "${RESULT_CSVS[@]}"; do
    echo "  - $csv"
    echo "    Metrics: ${csv%.csv}_metrics.json"
    echo "    Video:   ${csv%.csv}_3d.mp4"
done

# Generate comparison table
echo ""
echo "========================================================================"
echo "Generating comparison table..."
echo "========================================================================"

python3 - <<'EOF'
import json
import os
import sys

results_dir = "/home/lxx/LPV_ws/src/lpv_attack_sim/results"
scenarios = ["mild", "moderate", "severe"]

print("\n" + "="*90)
print("GPS SPOOFING ATTACK - SCENARIO COMPARISON TABLE")
print("="*90)
print()

# Header
print(f"{'Metric':<35} | {'Mild':>12} | {'Moderate':>12} | {'Severe':>12}")
print("-" * 90)

# Find metrics files
metrics_data = {}
for scenario in scenarios:
    pattern = f"gps_spoof_attack_*_{scenario}_metrics.json"
    import glob
    files = glob.glob(os.path.join(results_dir, pattern))
    if files:
        with open(files[0], 'r') as f:
            metrics_data[scenario] = json.load(f)

if not metrics_data:
    print("No metrics files found!")
    sys.exit(1)

# Extract and display key metrics
def get_metric(data, path):
    """Navigate nested dict with path like 'attack.max_deviation_m'"""
    keys = path.split('.')
    val = data
    for k in keys:
        val = val.get(k, None)
        if val is None:
            return None
    return val

metrics_to_compare = [
    ('Baseline mean error (m)', 'baseline.mean_tracking_error_m', '.3f'),
    ('Attack duration (s)', 'attack.duration_s', '.2f'),
    ('Max deviation (m)', 'attack.max_deviation_m', '.3f'),
    ('Mean deviation (m)', 'attack.mean_deviation_m', '.3f'),
    ('Final deviation (m)', 'attack.final_deviation_m', '.3f'),
    ('Max acceleration (m/s²)', 'attack.max_acceleration_m_s2', '.3f'),
    ('Recovery time (s)', 'post_attack.recovery_time_s', '.2f'),
    ('Final error after recovery (m)', 'post_attack.final_error_m', '.3f'),
    ('Attack effectiveness (×baseline)', 'summary.attack_effectiveness', '.2f'),
]

for label, path, fmt in metrics_to_compare:
    row = f"{label:<35} |"
    for scenario in scenarios:
        val = get_metric(metrics_data[scenario], path)
        if val is not None:
            row += f" {val:>12{fmt}} |"
        else:
            row += f" {'N/A':>12} |"
    print(row)

print("="*90)
print()
EOF

echo ""
echo "Batch comparison complete!"
