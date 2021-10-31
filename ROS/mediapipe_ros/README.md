# mediapipe_ros

## Environment

* Ubuntu 20.04
* ROS1 noetic
* python 3
* Intel Realsense D435

## How to build

```bash
mkdir -p ws/src && cd ws/src
wget xxxxx
wstool install .
cd ../
source /opt/ros/noetic/setup.bash
rosdep install  --from-paths src
catkin build
```

## How to launch

```bash
source devel/setup.bash
roslaunch mediapipe_ros pose_estimation.launch
```
