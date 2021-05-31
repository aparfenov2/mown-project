# Task type to go to certain pose
string GO_TO_POSE_TASK=go_to_pose
# Task type to explore polygon
string EXPLORE_POLYGON_TASK=polygon_task

std_msgs/Header header
string task_type
geometry_msgs/Pose target_pose
geometry_msgs/Polygon target_polygon
