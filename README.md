# Task perception

## Requirements
- skin_segmentation
- skin_segmentation_msgs
- nerf_b (modified to use hand segmentation)
- dbot (modified)
- dbot_ros (modified)

## Demonstration procedure
- [ ] Start up robot
- [ ] Verify workspace and person are visible (up to neck, 30 degree head tilt works well)
- [ ] Record data: `roslaunch task_perception record_data.launch`
- [ ] Check amount of time to trim from beginning and end: `rosed task_perception process_bag.launch`
- [ ] Process bag: `roslaunch task_perception process_bag.launch input:=/path/to/INPUT.bag output:=/path/to/OUTPUT.bag`

## Object models
This package requires object models.
For compatibility with `dbot_ros`, all object models must be in `.obj` format.
You should run `/usr/bin/pcl_mesh_sampling MODEL.obj MODEL.pcd`, most likely with the default settings.
All object meshes go in a ROS package called `object_meshes` and in a folder named `object_models`.

## Launch procedure
- [ ] `roscore`
- [ ] Frontend: `cd frontend; polymer serve;`
- [ ] Upload PR2 (only needed once): `roslaunch task_perception upload_pr2.launch`
- [ ] Backend: `roslaunch task_perception task_perception.launch --screen`
- [ ] RViz: `rosrun rviz rviz -d config/task_perception.rviz`
- [ ] Skin segmentation: `setvenv tf; setws tracking; roslaunch skin_segmentation service_test.launch  --screen`
- [ ] Skeleton tracking frontend: `setws tracking; roscd skin_segmentation/frontend; polymer serve`
