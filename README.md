# Software for the Pumas Humanoid Robot

This repository contains the software developed for the humanoid robot of the Biorrobotics Lab. 
![Humanoid photo](https://github.com/mnegretev/Humanoids/blob/eed79a5c102148e2fad110b6e99f638442f69420/Documentation/Images/HumanoidPhoto.png)

## Getting Started

### Prerrequisites

* Ubuntu 18.04
* ROS Melodic
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
