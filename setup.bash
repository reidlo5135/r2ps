#!/bin/bash
ros_packages=("r2ps_msgs" "r2ps_behavior_controller" "r2ps_process_controller")
for ros_package in "${ros_packages[@]}"; do
echo "========== [Source] $ros_package =========="
source ./${ros_package}/install/local_setup.bash
done
