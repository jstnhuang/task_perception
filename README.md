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
