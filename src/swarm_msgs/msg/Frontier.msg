geometry_msgs/Point  centroid       # world frame
uint32               cluster_size   # number of frontier pixels in cluster
float32              info_score     # cluster_size / path_cost (filled by coordinator)
string               assigned_to    # robot_id or "" if unassigned
