# Output of person_follower_node's state machine. The ROS2 analog of
# monocular_person_following's Target.msg.

std_msgs/Header header

# One of: "SEARCH", "TRACKING", "LOST", "REACQUIRE"
string state

# id of the currently locked/being-reacquired target track, or -1 if none.
int64 target_id

# All track ids visible this frame, and (during REACQUIRE) each one's
# appearance-similarity score against the stored target appearance.
# Empty/unused outside REACQUIRE.
int64[] track_ids
float32[] reid_scores

# SEARCH-state imprint-in-progress info: the track currently being held
# as a candidate (centered + within imprint_max_distance) and how long
# it's been held towards imprint_hold_time. candidate_id is -1 when
# nothing currently qualifies. Meaningless outside SEARCH.
int64 candidate_id
float32 candidate_hold_elapsed
float32 candidate_hold_required
