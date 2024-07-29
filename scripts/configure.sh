#!/bin/bash

apt_packages=("nodejs" "npm")

for package in "${apt_packages[@]}"; do
  echo "========== [Install] - [$package] =========="
  sudo apt-get install -y $package || exit
done

echo "========== [Install] - [n(nodejs)] =========="
sudo npm i -g n
sudo n lts

echo "====== [Compile] - [r2ps_msgs] ======"
cd ./r2ps_msgs
colcon build --symlink-install
cd ../

echo "========== [Create] setup.bash =========="
echo '#!/bin/bash' > ./scripts/setup.bash
echo 'source ./r2ps_msgs/install/local_setup.bash' >> ./scripts/setup.bash

echo "---------------------"
cat ./scripts/setup.bash
echo "---------------------"

node_packages=("r2ps_behavior_controller" "r2ps_process_controller")

for node_package in "${node_packages[@]}"; do
    echo "========== [Install] - [$node_package] =========="
    cd "./$node_package" || { echo "Directory $node_package not found"; exit 1; }
    npm install || { echo "npm install failed in $node_package"; exit 1; }
    npm run build || { echo "npm run build failed in $node_package"; exit 1; }
    cd ../
done

npm run g-msgs

echo "========== Configure Finished =========="