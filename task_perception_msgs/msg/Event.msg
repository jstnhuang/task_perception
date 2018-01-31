# Events have the following semantics:
# 
# SET_SKELETON_STATE: Sets the joint states of the Nerf skeleton tracker. This
#   is used to initialize or correct the skeleton.
# GRASP: Indicates that the person grasped an object. The hand is assumed to
#   hold on to the object until an UNGRASP event occurs with the same hand and
#   object. 
# UNGRASP: Indicates that the person let go of an object.
# SPAWN_OBJECT: Indicates that an object should start being tracked.
# UNSPAWN_OBJECT: Indicates that an object should stop being tracked.
# SET_OBJECT_POSE: Gives an object's new pose. This is used to initialize the
#   object's position, and to track the object from when it is grasped to when
#   a different object is grasped. This is only valid between SPAWN and UNSPAWN.

# 0-indexed video frame number for this event. All events must have this.
int32 frame_number 

string SET_SKELETON_STATE = set skeleton state
string GRASP = grasp
string UNGRASP = ungrasp
string SPAWN_OBJECT = spawn object
string UNSPAWN_OBJECT = unspawn object
string SET_OBJECT_POSE = set object pose
string type

# Usage of the remaining fields depends as follows:
# SET_SKELETON_STATE: nerf_joint_states must be set with all the joint values
#   from the Nerf tracker or control panel UI.
# GRASP: object_name and hand must be specified. The object must be SPAWNED
#   before this event.
# UNGRASP: hand must be specified. The hand must have previously GRASPED an
#   object. 
# SPAWN_OBJECT: object_name and object_mesh_path must be specified. The object
#   (as identified by the name) must not have been SPAWNED before, unless it was
#   UNSPAWNED.
# UNSPAWN_OBJECT: object_name must be specified. The object must have been
#   SPAWNED before.
# SET_OBJECT_POSE: object_name and object_pose must be specified. The object must
#   have been SPAWNED before.

skin_segmentation_msgs/NerfJointStates nerf_joint_states

string object_name
string object_mesh # E.g, bowl_1k.obj, must be in package://object_meshes/object_models
geometry_msgs/Pose object_pose # Object pose is relative to the camera frame

string LEFT_HAND = left hand
string RIGHT_HAND = right hand
string hand
