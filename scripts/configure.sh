#!/bin/bash

source /opt/ros/humble/setup.bash

ros_packages=("r2ps_msgs" "r2ps_utils" "r2ps_behavior_controller" "r2ps_process_controller")

for ros_package in "${ros_packages[@]}"; do
    echo "========== [Compile] - [$ros_package] =========="
    cd "./$ros_package" || { echo "Directory $ros_package not found"; exit 1; }
    colcon build --symlink-install || { echo "Compile Error in $ros_package"; exit 1; }
    source install/local_setup.bash
    cd ../
done

echo '========== [Create] setup.bash =========='
echo '#!/bin/bash' > ./scripts/setup.bash
echo 'ros_packages=("r2ps_msgs" "r2ps_utils" "r2ps_behavior_controller" "r2ps_process_controller")' >> ./scripts/setup.bash
echo 'for ros_package in "${ros_packages[@]}"; do' >> ./scripts/setup.bash
echo 'echo "========== [Source] $ros_package =========="' >> ./scripts/setup.bash
echo 'source ./${ros_package}/install/local_setup.bash' >> ./scripts/setup.bash
echo '========== [Source] setup.bash =========='
echo 'done' >> ./scripts/setup.bash

echo "---------------------"
cat ./scripts/setup.bash
echo "---------------------"

echo "========== Configure Finished =========="