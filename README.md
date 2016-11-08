# disparity_to_point_cloud

A ROS node to compute a point cloud from a disparity map.

## Installation

The disparity_to_point_cloud depends on the following packages:
- cv_bridge
```bash
sudo apt-get install ros-$ROS_DISTRO-cv-bridge
```
- pcl: follow the instruction [here](http://pointclouds.org/downloads/linux.html)
- catkin build: follow the instruction [here](http://catkin-tools.readthedocs.io/en/latest/installing.html)

Clone [this](https://github.com/simonegu/disparity_to_point_cloud) git repositories into your `catkin_ws/src` directory.

You are now ready to build the package with:
```bash
cd catkin_ws
catkin build
```

## Configuration
You need to change the input remap in the launch file for your images and point cloud topics:
- disparity image: https://github.com/PX4/disparity_to_point_cloud/blob/master/launch/d2pcloud.launch#L3
- point cloud : https://github.com/PX4/disparity_to_point_cloud/blob/master/launch/d2pcloud.launch#L4

## Use
After starting your ```roscore``` use this to launch disparity_to_point_cloud:
```bash
roslaunch disparity_to_point_cloud d2pcloud.launch
```
