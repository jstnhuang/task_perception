# Task perception

## Requirements
- PCL 1.8
- skin_segmentation
- skin_segmentation_msgs
- nerf_b (modified to use hand segmentation)
- dbot
- dbot_ros

## Building
To build with PCL 1.8:
- Download, build, and install PCL 1.8 from source
- Add pcl_conversions to your catkin workspace and change `find_package(PCL)` to `find_package(PCL 1.8)`

## Object models
This package requires object models.
For compatibility with `dbot_ros`, all object models must be in `.obj` format.
You should run `/usr/bin/pcl_mesh_sampling MODEL.obj MODEL.pcd`, most likely with the default settings.
All object meshes go in a ROS package called `object_meshes` and in a folder named `object_models`.
