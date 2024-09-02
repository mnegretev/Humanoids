# Software for the Pumas Humanoid Robot

This repository contains the software developed for the humanoid robot of the Biorrobotics Lab. 
![Humanoid photo](https://github.com/mnegretev/Humanoids/blob/eed79a5c102148e2fad110b6e99f638442f69420/Documentation/Images/HumanoidPhoto.png)

## Getting Started

### Prerrequisites

* [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (`ros-noetic-desktop-full` required)
* Gazebo (included in `ros-noetic-desktop-full`)
* RViz (included in `ros-noetic-desktop-full`)
* A lot of patience

### Installation

```
 cd Humanoids
 ./Setup.sh -i
 cd catkin_ws
 catkin_make -j1 -l1
```
## Testing
```
 source devel/setup.bash
 roslaunch surge_et_ambula humanoid_simul.launch
```
