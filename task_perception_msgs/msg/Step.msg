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
string type

# If action_type == GRASP:
# - object_state: the object state at the time of the demonstration
# - wrist_pose: the pose of the person's wrist at the time of the grasp,
#     relative to the object frame
# - object_trajectory: unused
# - times_from_start: unused

# If action_type == UNGRASP:
# - object_state: unused
# - wrist_pose: unused
# - object_trajectory: unused
# - times_from_start: unused

# If action_type == FOLLOW_TRAJECTORY:
# - object_state: the object state at the beginning of the demonstrated trajectory
# - wrist_pose: unused
# - object_trajectory: the trajectory of the object relative to its initial pose
# - times_from_start: timing information for the trajectory

task_perception_msgs/ObjectState object_state
geometry_msgs/Pose wrist_pose

# Times for each pose in the ee_trajectory from the start of this step
# The trajectory will be retimed as needed for the robot.
geometry_msgs/Pose[] object_trajectory
duration[] times_from_start 
