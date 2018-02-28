# Represents the state of a hand

string GRASPING = grasping
string PUSHING = pushing
string NONE = none
string current_action

# If current_action is not NONE, then both of these must be specified
string object_name # The name of the object being contacted
geometry_msgs/Pose wrist_pose # Pose of the person's wrist relative to object.
