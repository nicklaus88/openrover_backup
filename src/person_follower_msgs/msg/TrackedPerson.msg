# A person track with a persistent ID, maintained by person_tracker_node.
# The ROS2 analog of monocular_people_tracking's Track message.

int64 id

float32[17] keypoints_x
float32[17] keypoints_y
float32[17] keypoints_score

float32 bbox_x
float32 bbox_y
float32 bbox_w
float32 bbox_h

# Kalman-filtered 3D position/velocity in the camera optical frame.
geometry_msgs/Point position
geometry_msgs/Vector3 velocity

# Same position transformed into base_link, used by robot_controller_node.
# position_base_link.z is meaningless (planar transform); only x,y are used.
geometry_msgs/Point position_base_link
bool has_position_base_link

# Detection confidence carried over from the last matched detection.
float32 score

# Number of consecutive frames without a matching detection (0 = matched
# this frame). Tracks are dropped by person_tracker_node once this exceeds
# a threshold.
int32 missed_frames

# Frames this track has existed for, matched or not.
int32 age
