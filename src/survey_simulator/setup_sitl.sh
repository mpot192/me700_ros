#!/bin/bash
source ~/.bashrc

SRC_DIR=$1/PX4-Autopilot
BUILD_DIR=$1/PX4-Autopilot/build/px4_sitl_default

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${BUILD_DIR}/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${SRC_DIR}/Tools/sitl_gazebo/models:$1/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${BUILD_DIR}/build_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${SRC_DIR}
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${SRC_DIR}/Tools/sitl_gazebo

roslaunch survey_simulator sitl_mavros.launch sdf:=$2 world:=$3