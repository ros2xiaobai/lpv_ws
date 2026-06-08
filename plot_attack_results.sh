#!/usr/bin/env bash
set -e

source "$HOME/LPV_ws/setup_lpv_env.bash"
rosrun lpv_attack_sim plot_attack_results.py
