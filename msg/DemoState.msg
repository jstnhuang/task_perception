# Represents the full state of a demonstration for a single video frame.

time stamp # Time stamp when the depth data was acquired
skin_segmentation_msgs/NerfJointStates nerf_joint_states
task_perception_msgs/ObjectState[] object_states
task_perception_msgs/HandState left_hand
task_perception_msgs/HandState right_hand
