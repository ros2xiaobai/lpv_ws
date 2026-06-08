#!/usr/bin/env bash
set -euo pipefail

RESULTS_DIR="/home/lxx/LPV_ws/src/lpv_attack_sim/results"
STAMP="$(date +%Y%m%d_%H%M%S)"
GAZEBO_VIDEO="${RESULTS_DIR}/gazebo_window_attack_record_${STAMP}.mp4"
LOG_FILE="${RESULTS_DIR}/gazebo_window_attack_record_${STAMP}.log"

mkdir -p "${RESULTS_DIR}"
cd /home/lxx/PX4_Firmware

rm -rf /home/lxx/.ros/sitl_iris_0 \
       /home/lxx/PX4_Firmware/build/px4_sitl_default/tmp/rootfs/eeprom/parameters_10016

source /home/lxx/LPV_ws/setup_lpv_env.bash

roslaunch lpv_attack_sim single_attack_deviation.launch gui:=true record_bag:=false >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  set +e
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null
    wait "${LAUNCH_PID}" 2>/dev/null
  fi
}
trap cleanup EXIT

line=""
for _ in $(seq 1 90); do
  line="$(wmctrl -lG | awk 'tolower($0) ~ /gazebo/ {line=$0} END {print line}')"
  if [[ -n "${line}" ]]; then
    break
  fi
  sleep 1
done

if [[ -z "${line}" ]]; then
  echo "ERROR: Gazebo window not found" >&2
  exit 1
fi

wid="$(awk '{print $1}' <<<"${line}")"
x="$(awk '{print $3}' <<<"${line}")"
y="$(awk '{print $4}' <<<"${line}")"
w="$(awk '{print $5}' <<<"${line}")"
h="$(awk '{print $6}' <<<"${line}")"

screen="$(xdpyinfo | awk '/dimensions:/ {print $2; exit}')"
sw="${screen%x*}"
sh="${screen#*x}"
sh="${sh%%[^0-9]*}"

maxw=$((sw - x))
maxh=$((sh - y))
if (( w > maxw )); then w="${maxw}"; fi
if (( h > maxh )); then h="${maxh}"; fi
w=$((w / 2 * 2))
h=$((h / 2 * 2))

wmctrl -ia "${wid}" || true
wmctrl -ir "${wid}" -b add,above || true
sleep 1

echo "Recording Gazebo window ${wid} at ${w}x${h}+${x},${y} -> ${GAZEBO_VIDEO}"
ffmpeg -y -f x11grab -framerate 20 -video_size "${w}x${h}" \
  -i ":0.0+${x},${y}" -t 82 -codec:v libx264 -preset veryfast \
  -pix_fmt yuv420p "${GAZEBO_VIDEO}"

wait "${LAUNCH_PID}" || true
trap - EXIT

CSV_PATH="$(ls -t "${RESULTS_DIR}"/setpoint_fdi_attack_*.csv | head -n 1)"
PLOT_VIDEO="${CSV_PATH%.csv}_csv_plot.mp4"
python3 /home/lxx/LPV_ws/src/lpv_attack_sim/scripts/make_attack_video.py \
  --csv "${CSV_PATH}" \
  --output "${PLOT_VIDEO}" \
  --fps 20 \
  --duration 28

echo "GAZEBO_VIDEO=${GAZEBO_VIDEO}"
echo "CSV_PATH=${CSV_PATH}"
echo "PLOT_VIDEO=${PLOT_VIDEO}"
