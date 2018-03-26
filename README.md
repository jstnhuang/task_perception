# Task perception

## Requirements
- dbot_ros_msgs
- object_meshes
- skin_segmentation_msgs
- [rapid](https://github.com/jstnhuang/rapid)

Build in a separate workspace called **tracking**:
- skin_segmentation

Buid these in a separate workspace called **dbot**:
- nerf_b
- dbot (modified)
- dbot_ros (modified)

## Demonstration procedure
- [ ] Start up cameras
  - If tripod: `roslaunch task_perception xtion_tripod.launch`
  - If PR2: start up as usual
- [ ] Verify workspace and person are visible (up to neck, 30 degree head tilt works well)
- [ ] Record data
  - If tripod: `roslaunch task_perception record_demonstration.launch is_pr2:=false`
  - If PR2: `roslaunch task_perception record_demonstration.launch is_pr2:=true`
- [ ] Check amount of time to trim from beginning and end: `rosed task_perception process_bag.launch`
- [ ] Process bag: `roslaunch task_perception process_bag.launch input:=/path/to/INPUT.bag output:=/path/to/OUTPUT.bag`

## Object models
This package requires object models.
For compatibility with `dbot_ros`, all object models must be in `.obj` format.
You should run `/usr/bin/pcl_mesh_sampling MODEL.obj MODEL.pcd`, most likely with the default settings.
All object meshes go in a ROS package called `object_meshes` and in a folder named `object_models`.

## Annotation procedure
- [ ] `roscore`
- [ ] Frontend: `cd frontend; polymer serve;`
- [ ] Upload PR2 (only needed once): `roslaunch task_perception upload_pr2.launch`
- [ ] Add an object tracker for each object in the demonstration: `rosed task_perception dbot_nerf.launch`
- [ ] In dbot/nerf workspace, run skeleton/object tracking: `cd task_perception/launch; setws dbot; roslaunch dbot_nerf.launch`
- [ ] Backend: `roslaunch task_perception task_perception.launch --screen`
- [ ] RViz: `rosrun rviz rviz -d task_perception/config/task_perception.rviz`
- [ ] Skin segmentation: `setvenv tf; setws tracking; roslaunch skin_segmentation service_test.launch --screen`
- [ ] Skeleton tracking frontend: `setws tracking; roscd skin_segmentation/frontend; polymer serve`

## Imitation procedure
- [ ] Start up robot
  - If simulation: start Gazebo and MoveIt: `roslaunch task_imitation pr2_sim.launch`
  - If PR2: Start RWS and `rosrun rqt_console rqt_console`
- [ ] Raise torso to 0.4 m
- [ ] RViz: `rosrun rviz rviz -d task_imitation/config/imitation.rviz`
- [ ] Move to start position
  - If simulation: use MoveIt
  - If PR2: use Rapid PbD
- [ ] Run dbot object initializer: `cd task_imitation/launch; setws dbot; roslaunch dbot.launch`
- [ ] Program server: `roslaunch task_imitation task_imitation.launch --screen`
- [ ] Choose which task to imitate: `rosed task_imitation test_imitation.py`
- [ ] Trigger processing/execution of a bag file: `rosrun task_imitation test_imitation.py`
