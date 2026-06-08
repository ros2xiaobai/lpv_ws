#!/usr/bin/env bash
set -e

source "$HOME/LPV_ws/setup_lpv_env.bash"
roslaunch lpv_attack_sim single_attack_deviation.launch start_px4:=false record_bag:=true
