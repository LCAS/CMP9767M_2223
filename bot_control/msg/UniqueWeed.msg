# Defines a unique weed with an ID
# Also contains a confidence count (how many times a weed is registered)
uint8[16] uuid
uint32 confidence
geometry_msgs/PointStamped point