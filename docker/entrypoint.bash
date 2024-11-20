#!/bin/bash

# Setup ROS 2  environment
source /opt/ros/humble/setup.bash
source /home/user/ros/install/setup.bash
source /astro_ws/install/setup.bash
export NODE_PATH=/usr/lib/node_modules
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
npx generate-ros-messages
cd /dev/shm
chmod 777 -R .
# export NVM_DIR="$HOME/.nvm"
# [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" [ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"
exec "$@"
