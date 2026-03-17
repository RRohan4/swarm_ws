std_msgs/Header header
string               robot_id
uint8                state          # EXPLORING=0  WAITING=1  DONE=2
geometry_msgs/Pose   pose
uint32               map_cells_known
uint32               frontiers_remaining
