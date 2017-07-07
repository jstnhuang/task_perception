# Open a bag file
string OPEN_BAG = open bag
# Scrub to a specific time in the bag file
string VIEW_TIME = view time
string type

string bag_path # If type is OPEN, this specifies the path of the bag file
duration time # If type is VIEW_TIME, this specifies the time to scrub to
