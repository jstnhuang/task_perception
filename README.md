# Task perception

## Requirements
- skin_segmentation
- skin_segmentation_msgs
- nerf_b (modified to use hand segmentation)
- dbot
- dbot_ros

## Object models
This package requires object models.
For compatibility with `dbot_ros`, all object models must be in `.obj` format.
You should run `/usr/bin/pcl_mesh_sampling MODEL.obj MODEL.pcd`, most likely with the default settings.
All object meshes go in a ROS package called `object_meshes` and in a folder named `object_models`.
