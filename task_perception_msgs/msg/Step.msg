# Specifies the requested start time for this step, relative to the start time of the first step.
# The first step therefore has a start time of 0.
# The start time should leave enough time for the previous trajectory (on the same arm) to finish.
# If not, this step may be delayed.
duration start_time

# Specify which arm this applies to (left or right for bimanual robots, arm for single arm robots).
string LEFT = left
string RIGHT = right
string arm

string GRASP = grasp
string UNGRASP = ungrasp
string FOLLOW_TRAJECTORY = follow trajectory
string MOVE_TO_POSE = move to pose
string type

# Step parameters ###########################################################
# If action_type == GRASP:
# - object_state: the object state at the time of the demonstration
# - ee_trajectory: the first element is the transform of the grasp relative to the object
# - times_from_start: unused

# If action_type == UNGRASP:
# - object_state: name specifies the object that was ungrasped.
# - ee_trajectory: unused
# - times_from_start: unused

# If action_type == FOLLOW_TRAJECTORY:
# - object_state: the object state at the beginning of the demonstrated trajectory
# - ee_trajectory: the trajectory of the end effector relative to the object's initial pose, given in object_state
# - times_from_start: timing information for the trajectory

# If action_type == MOVE_TO_POSE:
# - object_state: the object to move relative to
# - ee_trajectory: the first element is the pose of the grasp relative to the pose given in object_state
# - times_from_start: the first element is the estimated time to move to this pose

task_perception_msgs/ObjectState object_state

geometry_msgs/Pose[] ee_trajectory

# Times for each pose in the ee_trajectory from the start of this step
# The trajectory will be retimed as needed for the robot.
duration[] times_from_start 
