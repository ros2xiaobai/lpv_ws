#!/bin/bash
# Convenient wrapper to run GPS spoofing attack with automatic config update

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_UPDATER="$SCRIPT_DIR/update_gps_spoof_config.py"

# Parse arguments
SCENARIO="moderate"  # default
GENERATE_METRICS="false"
GENERATE_VIDEO="false"

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Run GPS spoofing attack simulation with automatic configuration."
    echo ""
    echo "Options:"
    echo "  --scenario SCENARIO   Attack scenario (mild/moderate/severe/asymmetric)"
    echo "  --metrics             Generate quantitative metrics after simulation"
    echo "  --video               Generate 3D trajectory video after simulation"
    echo "  --all                 Generate both metrics and video"
    echo "  -h, --help            Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 --scenario severe --all"
    echo "  $0 --scenario mild --metrics"
    exit 0
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --scenario)
            SCENARIO="$2"
            shift 2
            ;;
        --metrics)
            GENERATE_METRICS="true"
            shift
            ;;
        --video)
            GENERATE_VIDEO="true"
            shift
            ;;
        --all)
            GENERATE_METRICS="true"
            GENERATE_VIDEO="true"
            shift
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
done

echo "========================================================================"
echo "GPS Spoofing Attack Simulation"
echo "========================================================================"
echo "Scenario: $SCENARIO"
echo ""

# Update configuration
echo "[Step 1/3] Updating SDF configuration..."
python3 "$CONFIG_UPDATER" --scenario "$SCENARIO"

if [ $? -ne 0 ]; then
    echo "Error: Failed to update configuration"
    exit 1
fi

# Clean up existing processes
echo ""
echo "[Step 2/3] Cleaning up existing processes..."
pkill -9 -f "gzserver|gzclient|px4|mavros|roslaunch|rosmaster" 2>/dev/null || true
sleep 3

# Launch simulation
echo ""
echo "[Step 3/3] Launching simulation..."
cd /home/lxx/LPV_ws
source devel/setup.bash
roslaunch lpv_attack_sim gps_spoof_deviation.launch

# Post-processing
if [ "$GENERATE_METRICS" = "true" ] || [ "$GENERATE_VIDEO" = "true" ]; then
    echo ""
    echo "========================================================================"
    echo "Post-Processing"
    echo "========================================================================"

    # Find latest CSV
    LATEST_CSV=$(ls -t src/lpv_attack_sim/results/gps_spoof_attack_*.csv 2>/dev/null | head -1)

    if [ -z "$LATEST_CSV" ] || [ ! -f "$LATEST_CSV" ]; then
        echo "Warning: No CSV file found, skipping post-processing"
        exit 0
    fi

    echo "Using CSV: $LATEST_CSV"

    if [ "$GENERATE_METRICS" = "true" ]; then
        echo ""
        echo "[Metrics] Generating quantitative metrics..."
        python3 "$SCRIPT_DIR/generate_attack_metrics.py" "$LATEST_CSV"
    fi

    if [ "$GENERATE_VIDEO" = "true" ]; then
        echo ""
        echo "[Video] Generating 3D trajectory video..."
        python3 "$SCRIPT_DIR/make_gps_spoof_video_3d.py" --csv "$LATEST_CSV"
    fi
fi

echo ""
echo "========================================================================"
echo "Simulation complete!"
echo "========================================================================"
