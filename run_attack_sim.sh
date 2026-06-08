#!/usr/bin/env bash
set -e

source "$HOME/LPV_ws/setup_lpv_env.bash"
roslaunch lpv_attack_sim single_attack_deviation.launch gui:=true record_bag:=true
