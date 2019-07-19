# Task perception

## Requirements
- [rapid](https://github.com/jstnhuang/rapid)

Build in a separate workspace called **tracking**:
- [skin_segmentation](https://github.com/jstnhuang/skin_segmentation)
- [skin_segmentation_msgs](https://github.com/jstnhuang/skin_segmentation_msgs)

Buid these in a separate workspace called **dbot**:
- nerf_b (Contact Aaron Walsman for access)
- [dbot (modified version)](https://github.com/jstnhuang/dbot)
- [dbot_ros (modified version)](https://github.com/jstnhuang/dbot_ros)
- [dbot_ros_msgs](https://github.com/jstnhuang/dbot_ros_msgs)

## Demonstration procedure
- [ ] `roscore`
- [ ] RViz: `rosrun rviz rviz -d task_perception/config/record_demonstration.rviz`
- [ ] Start up cameras
  - If tripod: `roslaunch task_perception xtion_tripod.launch`
  - If PR2: start up as usual
- [ ] Verify workspace and person are visible (up to neck, 30 degree head tilt works well)
- [ ] (Optional): Test object tracking
  - [ ] Specify the object to track: `setws dbot; roscd dbot_ros; vim config/object.yaml`
  - [ ] Run the tracker: `roslaunch dbot_ros particle_tracker.yaml`
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
- [Preparing object models](https://github.com/jstnhuang/task_perception/wiki/Preparing-object-models)

## Annotation procedure
- [ ] `roscore`
- [ ] Frontend: `cd frontend; polymer serve;`
- [ ] Add an object tracker for each object in the demonstration: `cd task_perception/launch; vim dbot_nerf.launch`
- [ ] In dbot/nerf workspace, run skeleton/object tracking: `cd task_perception/launch; setws dbot; roslaunch dbot_nerf.launch`
- [ ] Backend: `roslaunch task_perception task_perception.launch --screen`
- [ ] RViz: `rosrun rviz rviz -d task_perception/config/task_perception.rviz`
- [ ] Skin segmentation: `setvenv tf; setws tracking; roslaunch skin_segmentation service_test.launch --screen`
- [ ] Skeleton tracking frontend: `setws tracking; roscd skin_segmentation/frontend; polymer serve`

## Imitation procedure
Save a point cloud in a real scene using: `rosrun rapid_perception save_cloud NAME base_footprint cloud_in:=/head_mount_kinect/depth_registered/points`

- [ ] Start up robot
  - If simulation: start Gazebo and MoveIt: `roslaunch task_imitation pr2_sim.launch`
  - If PR2: Start RWS and `rosrun rqt_pr2_dashboard rqt_pr2_dashboard`
- [ ] Raise torso to 0.4 m: `rosrun rapid_pr2 torso 0.4`
- [ ] RViz: `rosrun rviz rviz -d task_imitation/config/imitation.rviz`
- [ ] Move to start position
  - If simulation: use MoveIt
  - If PR2: use Rapid PbD
- [ ] Run dbot object initializer: `cd task_imitation/launch; setws dbot; roslaunch dbot.launch`
- [ ] Program server
  - If simulation: `roslaunch task_imitation task_imitation.launch bag:=/path/to/NAME.bag --screen`
  - If PR2: `roslaunch task_imitation task_imitation.launch --screen`
- [ ] If PR2: Start local websocket server: `setws catkin; setrobot c1; roslaunch task_imitation rosbridge_websocket_local.launch`
- [ ] Start the frontend: `roscd task_perception/frontend; polymer serve`
- [ ] Visit http://localhost:8081/imitation and enter the bag file of the demonstration to execute
  - If PR2: When the websocket dialog appears, change the port to 9091.
