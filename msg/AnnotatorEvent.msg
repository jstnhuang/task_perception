# Open a bag file
string OPEN_BAG = open bag
# Step to the next frame
string STEP = step
# Add an object to track, depth_frame indicates when to start.
string ADD_OBJECT = add object
# After making manual adjustments to the skeleton, the user can trigger
# SAVE_SKELETON to save the joint state to the annotation.
string SAVE_SKELETON = save skeleton
string type

string bag_path # If type is OPEN_BAG, this specifies the path of the bag file
string mesh_name # Name of .obj mesh file to track.
