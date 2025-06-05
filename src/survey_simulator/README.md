# Survey Simulator

This repository contains a number of scripts and launch files for easy setup of PX4's SITL with ROS1 integration in a catkin workspace.

## Prerequisites
While this repository should remain compatible with other versions of ROS1/PX4, it has been developed and tested with the following:

 - Ubuntu 20.04.3
 - ROS1 Noetic
 - PX4 SITL v1.12.3

## Installation
This (and the rest of the survey series packages) is designed to be used with the catkin build system. You will need to install some extra dependencies on top of a clean ROS install

```
sudo apt install python3-catkin-pkg python3-catkin-tools
```

Since MAVROS and geographiclib is not installed as part of a base ROS install

```
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh
```

If you haven't yet done so, setup a new catkin workspace

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin confg -DCMAKE_BUILD_TYPE=Release
```

Clone this repository 

```
cd ~/catkin_ws/src
git clone git@github.com:tlin442/survey_simulator.git
```

You will need to build PX4 SITL the first time you run this repo. The easy way to do this is to use the init script provided

```
cd survey_simulator
sudo ./init_sitl.sh
```

Finally, build the catkin workspace to gain access to launch files

```
cd ~/catkin_ws
catkin build
```

## Running the Simulation
If you have built SITL and the catkin workspace correctly, you should be able to start a SITL session with ROS using

```
source devel/setup.bash && roslaunch survey_simulator run_sitl.launch
```

To run a simulation using a forest world instead, run

```
source devel/setup.bash && roslaunch survey_simulator run_sitl_forest.launch
```

Do note that running SITL and additional nodes are quite resource intensive, so running this directly on a reasonably powerful machine instead of a VM is highly recommended.