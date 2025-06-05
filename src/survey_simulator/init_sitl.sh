#!/bin/bash

# Init submodules and dependencies
git submodule update --init --recursive

# Setup build tools
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# Build SITL for gazebo/ROS
cd PX4-Autopilot
DONT_RUN=1 make px4_sitl gazebo
