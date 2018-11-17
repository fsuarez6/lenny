# PET types
string PET_COLOR=PET_COLOR
string PET_TRANSPARENT=PET_TRANSPARENT
# HDPE types
string HDPE_COLOR=HDPE_COLOR
string HDPE_WHITE=HDPE_WHITE

std_msgs/Header header

# Classification results
string      bottle_type
float64     score

# Bottle transform (If needed, we can include covariance details)
string                      frame_id
string                      parent_frame_id
geometry_msgs/Transform     transform

# Bounding box size
geometry_msgs/Vector3 bbox_size
