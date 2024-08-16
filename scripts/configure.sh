#!/bin/bash

source /opt/ros/humble/setup.bash

setup_packages=("r2ps_msgs" "r2ps_utils")

for setup_packages in "${setup_package[@]}"; do
    echo "========== [Compile] - [$setup_package] =========="
    colcon build --packages-select ${setup_package} || { echo "Compile Error in $ros_package"; exit 1; }
    source install/setup.bash
done

source install/setup.bash
colcon build --symlink-install
source install/local_setup.bash

echo "========== Configure Finished =========="