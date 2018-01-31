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
string action_type

# If action_type == GRASP:
# - object_state: the object we are grasping
# - ee_trajectory: the first element is the transform of the grasp relative to the object
# - times_from_start: unused

# If action_type == UNGRASP:
# - object_state: unused
# - ee_trajectory: unused
# - times_from_start: unused

# If action_type == FOLLOW_TRAJECTORY:
# - object_state: initial state of the object
# - ee_trajectory: is an end-effector trajectory relative to the initial object pose
# - times_from_start: timing information for the start is 

task_perception_msgs/ObjectState object_state

# Times for each pose in the ee_trajectory from the start of this step
# The trajectory will be retimed as needed for the robot.
geometry_msgs/Pose[] ee_trajectory
duration[] times_from_start 
