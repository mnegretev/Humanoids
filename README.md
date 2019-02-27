# Software for the Pumas Humanoid Robot

This repository contains the software developed for the humanoid robot of the Biorrobotics Lab. 

## Getting Started

### Prerrequisites

* Ubuntu 16.04
* ROS Kinectic
* Gazebo
* A lot of patience

### Installation

```
 cd ~/Humanoids
 ./Setup.sh -i
 cd catkin_ws
 catkin_make -j1 -l1
```
## Testing
```
 roslaunch surge_et_ambula humanoid_simul.launch
```
