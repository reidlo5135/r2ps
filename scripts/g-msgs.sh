#!/bin/bash

ros_packages=("r2ps_behavior_controller" "r2ps_process_controller")

echo "====== [Generate] - [generate ros messages] ======"

for ros_package in "${ros_packages[@]}"; do
    echo "========== [Generate] - [$ros_package] =========="
    cd "./$ros_package" || { echo "Directory $ros_package not found"; exit 1; }
    npx generate-ros-messages || { echo "Message Generation Failed in $ros_package"; exit 1; }
    cd ../
done