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
# - object_name: the name of the object we are grasping.
# - object_mesh: the mesh file of the object (given just as the file name,
#     not the full path, e.g., "pringles_1k.obj")
# - object_trajectory: unused
# - times_from_start: unused

# If action_type == UNGRASP:
# - object_name: unused
# - object_mesh: unused.
# - object_trajectory: unused
# - times_from_start: unused

# If action_type == FOLLOW_TRAJECTORY:
# - object_name: the name of the object we are currently moving
# - object_mesh: the mesh file of the object.
# - object_trajectory: the trajectory of the object relative to its initial pose
# - times_from_start: timing information for the trajectory

string object_name
string object_mesh

# Times for each pose in the ee_trajectory from the start of this step
# The trajectory will be retimed as needed for the robot.
geometry_msgs/Pose[] object_trajectory
duration[] times_from_start 
