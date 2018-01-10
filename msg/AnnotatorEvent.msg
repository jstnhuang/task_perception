# Open a bag file
string OPEN_BAG = open bag
# Step to the next frame
string STEP = step
# Add an object to track, mesh_name determines object model, object_name gives the user-friendly semantic name to the object.
string ADD_OBJECT = add object
# Only object_name need to be given. Adds an unspawn event.
string REMOVE_OBJECT = remove object
# After making manual adjustments to the skeleton, the user can trigger
# SAVE_SKELETON to save the joint state to the annotation.
string SAVE_SKELETON = save skeleton
# Step through the skeleton tracker once
string STEP_SKELETON = step skeleton
string DELETE_EVENT = delete event
string type

string bag_path # If type is OPEN_BAG, this specifies the path of the bag file
string mesh_name # Name of .obj mesh file to track.
string object_name # Semantic name given to a particular object
string event_type # The type of the event to be deleted
