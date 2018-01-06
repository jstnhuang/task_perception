string bag_path # Path to the bag file
int32 frame_count # Number of RGBD frames in the bag file
int32 current_frame # The current frame the annotator is on.
task_perception_msgs/Event[] events # The events that occur in the current frame.
